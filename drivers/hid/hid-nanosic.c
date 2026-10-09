// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nanosic WN8030 transport for Xiaomi piano.
 * HID transport follows the liuqin hid-nanosic driver. Firmware encoding
 * follows map220v's WN8030 implementation through hid-nanosic-protocol.h.
 * Report declarations match the piano stock module's HID interface.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

#include "hid-nanosic-protocol.h"

#define NANOSIC_VENDOR_ID	0x15d9
#define NANOSIC_APP_CMD_SIZE	66

/* Attach state byte of the keyboard's 0x38/0xa2 report. */
#define NANOSIC_ATTACH_CONNECTED	BIT(0)
#define NANOSIC_ATTACH_POWERED		BIT(1)
#define NANOSIC_ATTACH_TIMEOUT_MS	500

static const u8 nanosic_keyboard_rdesc[] = {
	0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x85, 0x05,
	0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00,
	0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
	0x81, 0x03, 0x95, 0x05, 0x05, 0x08, 0x19, 0x01,
	0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03,
	0x91, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00,
	0x26, 0xa4, 0x00, 0x05, 0x07, 0x19, 0x00, 0x2a,
	0xa4, 0x00, 0x81, 0x00, 0xc0,
};

static const u8 nanosic_consumer_rdesc[] = {
	0x05, 0x0c, 0x09, 0x01, 0xa1, 0x01, 0x85, 0x06,
	0x15, 0x00, 0x26, 0x80, 0x03, 0x19, 0x00, 0x2a,
	0x80, 0x03, 0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
	0xc0,
};

static const u8 nanosic_mouse_rdesc[] = {
	0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x85, 0x02,
	0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01,
	0x29, 0x05, 0x15, 0x00, 0x25, 0x01, 0x95, 0x05,
	0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x03,
	0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31,
	0x09, 0x38, 0x16, 0x00, 0x80, 0x26, 0xff, 0x7f,
	0x75, 0x10, 0x95, 0x03, 0x81, 0x06, 0xc0, 0xc0,
};

static const u8 nanosic_touchpad_rdesc[] = {
	0x05, 0x0d, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x19,
	0x15, 0x00, 0x25, 0x01, 0x35, 0x00, 0x45, 0x01,
	0x75, 0x01, 0x95, 0x02, 0x05, 0x09, 0x09, 0x01,
	0x09, 0x02, 0x81, 0x02, 0x95, 0x06, 0x81, 0x01,
	0x05, 0x0d, 0x09, 0x22, 0xa1, 0x02, 0x09, 0x42,
	0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x01,
	0x81, 0x02, 0x09, 0x32, 0x81, 0x02, 0x09, 0x47,
	0x81, 0x02, 0x95, 0x05, 0x81, 0x03, 0x75, 0x08,
	0x09, 0x51, 0x95, 0x01, 0x81, 0x02, 0x15, 0x00,
	0x26, 0xff, 0xff, 0x75, 0x10, 0x09, 0x30, 0x95,
	0x01, 0x81, 0x02, 0x05, 0x01, 0x15, 0x00, 0x26,
	0x7f, 0x0c, 0x75, 0x10, 0x55, 0x0f, 0x65, 0x11,
	0x09, 0x30, 0x35, 0x00, 0x46, 0x5e, 0x00, 0x81,
	0x02, 0x09, 0x31, 0x26, 0x57, 0x08, 0x46, 0x2a,
	0x00, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x05, 0x0d,
	0x09, 0x42, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x01, 0x81, 0x02, 0x09, 0x32, 0x81, 0x02,
	0x09, 0x47, 0x81, 0x02, 0x95, 0x05, 0x81, 0x03,
	0x75, 0x08, 0x09, 0x51, 0x95, 0x01, 0x81, 0x02,
	0x15, 0x00, 0x26, 0xff, 0xff, 0x75, 0x10, 0x09,
	0x30, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x15,
	0x00, 0x26, 0x7f, 0x0c, 0x75, 0x10, 0x55, 0x0f,
	0x65, 0x11, 0x09, 0x30, 0x35, 0x00, 0x46, 0x5e,
	0x00, 0x81, 0x02, 0x09, 0x31, 0x26, 0x57, 0x08,
	0x46, 0x2a, 0x00, 0x81, 0x02, 0xc0, 0xa1, 0x02,
	0x05, 0x0d, 0x09, 0x42, 0x15, 0x00, 0x25, 0x01,
	0x75, 0x01, 0x95, 0x01, 0x81, 0x02, 0x09, 0x32,
	0x81, 0x02, 0x09, 0x47, 0x81, 0x02, 0x95, 0x05,
	0x81, 0x03, 0x75, 0x08, 0x09, 0x51, 0x95, 0x01,
	0x81, 0x02, 0x15, 0x00, 0x26, 0xff, 0xff, 0x75,
	0x10, 0x09, 0x30, 0x95, 0x01, 0x81, 0x02, 0x05,
	0x01, 0x15, 0x00, 0x26, 0x7f, 0x0c, 0x75, 0x10,
	0x55, 0x0f, 0x65, 0x11, 0x09, 0x30, 0x35, 0x00,
	0x46, 0x5e, 0x00, 0x81, 0x02, 0x09, 0x31, 0x26,
	0x57, 0x08, 0x46, 0x2a, 0x00, 0x81, 0x02, 0xc0,
	0x05, 0x0d, 0x09, 0x54, 0x95, 0x01, 0x75, 0x08,
	0x15, 0x00, 0x25, 0x08, 0x81, 0x02, 0x09, 0x55,
	0xb1, 0x02, 0x06, 0x00, 0xff, 0x09, 0xc5, 0x15,
	0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x96, 0x00,
	0x01, 0xb1, 0x02, 0xc0,
};

struct nanosic_hid_info {
	const char *name;
	const u8 *rdesc;
	size_t rdesc_size;
	u16 product;
	u8 report_id;
};

static const struct nanosic_hid_info nanosic_hid_info[] = {
	{ "Xiaomi Keyboard", nanosic_keyboard_rdesc, sizeof(nanosic_keyboard_rdesc), 0x00a3, 0x05 },
	{ "Xiaomi Consumer", nanosic_consumer_rdesc, sizeof(nanosic_consumer_rdesc), 0x00a4, 0x06 },
	{ "Xiaomi Mouse", nanosic_mouse_rdesc, sizeof(nanosic_mouse_rdesc), 0x00a2, 0x02 },
	{ "Xiaomi Touch", nanosic_touchpad_rdesc, sizeof(nanosic_touchpad_rdesc), 0x00a1, 0x19 },
};

#define NANOSIC_HID_COUNT ARRAY_SIZE(nanosic_hid_info)

struct nanosic {
	struct i2c_client *client;
	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *wake_gpio;
	struct mutex lock;
	struct hid_device *hid[NANOSIC_HID_COUNT];
	struct input_dev *tablet;
	struct completion attach_known;
	u8 attach;
	bool tablet_registered;
	bool powered;
};

static int nanosic_write(struct nanosic *nano, u8 reg, const u8 *data, size_t len)
{
	u8 buffer[NANOSIC_FW_DATA_CMD_SIZE + 1];
	int ret;

	if (!len || len > NANOSIC_FW_DATA_CMD_SIZE)
		return -EINVAL;
	buffer[0] = reg;
	memcpy(buffer + 1, data, len);
	ret = i2c_master_send(nano->client, buffer, len + 1);
	if (ret != len + 1)
		return ret < 0 ? ret : -EIO;

	/* Stock bootloader writes have a 3 ms settling interval. */
	if (reg == NANOSIC_BOOT_REGISTER)
		fsleep(3000);
	return 0;
}

static int nanosic_read(struct nanosic *nano, u8 reg, u8 *data, u16 len)
{
	struct i2c_msg messages[] = {
		{ .addr = nano->client->addr, .len = 1, .buf = &reg },
		{ .addr = nano->client->addr, .flags = I2C_M_RD, .len = len, .buf = data },
	};
	int ret;

	ret = i2c_transfer(nano->client->adapter, messages, ARRAY_SIZE(messages));
	return ret == ARRAY_SIZE(messages) ? 0 : ret < 0 ? ret : -EIO;
}

static int nanosic_boot_read(struct nanosic *nano, u8 command, u8 *value)
{
	int ret;

	ret = nanosic_write(nano, NANOSIC_BOOT_REGISTER, &command, 1);
	if (ret)
		return ret;
	return nanosic_read(nano, NANOSIC_BOOT_REGISTER, value, 1);
}

static int nanosic_boot_state(struct nanosic *nano, u8 expected)
{
	u8 state;
	int ret;

	ret = nanosic_boot_read(nano, 0x05, &state);
	if (ret)
		return ret;
	if (state != expected) {
		dev_err(&nano->client->dev, "boot state %u, expected %u\n", state, expected);
		return -EPROTO;
	}
	return 0;
}

static int nanosic_boot_firmware(struct nanosic *nano, const struct nanosic_firmware *fw)
{
	u8 command[NANOSIC_FW_DATA_CMD_SIZE];
	u8 boot_id;
	u32 offset = 0;
	int ret, count;

	ret = nanosic_boot_read(nano, 0x04, &boot_id);
	if (ret)
		return ret;
	if (boot_id != 0xc8) {
		dev_err(&nano->client->dev, "unsupported boot ID %#x\n", boot_id);
		return -ENODEV;
	}
	ret = nanosic_boot_state(nano, 0);
	if (ret)
		return ret;

	nanosic_firmware_header(fw, command);
	ret = nanosic_write(nano, NANOSIC_BOOT_REGISTER, command, NANOSIC_FW_HEADER_CMD_SIZE);
	if (ret)
		return ret;
	ret = nanosic_boot_state(nano, 1);
	if (ret)
		return ret;

	while (offset < fw->size) {
		count = nanosic_firmware_chunk(fw, offset, command);
		if (count < 0)
			return count;
		ret = nanosic_write(nano, NANOSIC_BOOT_REGISTER, command, sizeof(command));
		if (ret)
			return ret;
		offset += count;
	}
	command[0] = 0x08;
	ret = nanosic_write(nano, NANOSIC_BOOT_REGISTER, command, 1);
	if (ret)
		return ret;
	ret = nanosic_boot_state(nano, 3);
	if (ret)
		return ret;
	command[0] = 0x09;
	return nanosic_write(nano, NANOSIC_BOOT_REGISTER, command, 1);
}

static int nanosic_app_command(struct nanosic *nano, u8 command, u8 value)
{
	u8 data[NANOSIC_APP_CMD_SIZE] = { 0x32, 0, 0x4e, 0x31, 0x80, 0x38 };
	unsigned int i;

	data[6] = command;
	data[7] = 1;
	data[8] = value;
	for (i = 2; i < 9; i++)
		data[9] += data[i];
	return nanosic_write(nano, NANOSIC_REPORT_REGISTER, data, sizeof(data));
}

static int nanosic_hid_start(struct hid_device *hid)
{
	return 0;
}

static void nanosic_hid_stop(struct hid_device *hid)
{
}

static int nanosic_hid_open(struct hid_device *hid)
{
	return 0;
}

static void nanosic_hid_close(struct hid_device *hid)
{
}

static int nanosic_hid_parse(struct hid_device *hid)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(nanosic_hid_info); i++)
		if (hid->product == nanosic_hid_info[i].product)
			return hid_parse_report(hid, nanosic_hid_info[i].rdesc,
						nanosic_hid_info[i].rdesc_size);
	return -ENODEV;
}

static int nanosic_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
				   u8 *buf, size_t len, unsigned char rtype, int reqtype)
{
	/* This is the descriptor's advertised bound, not a measured contact count. */
	if (hid->product == 0x00a1 && reportnum == 0x19 && len >= 2 &&
	    rtype == HID_FEATURE_REPORT && reqtype == HID_REQ_GET_REPORT) {
		buf[0] = 0x19;
		buf[1] = 8;
		return 2;
	}
	return -EOPNOTSUPP;
}

static const struct hid_ll_driver nanosic_hid_ll_driver = {
	.start = nanosic_hid_start,
	.stop = nanosic_hid_stop,
	.open = nanosic_hid_open,
	.close = nanosic_hid_close,
	.parse = nanosic_hid_parse,
	.raw_request = nanosic_hid_raw_request,
};

static void nanosic_destroy_hid(void *data)
{
	struct nanosic *nano = data;
	unsigned int i;

	for (i = 0; i < NANOSIC_HID_COUNT; i++) {
		if (!nano->hid[i])
			continue;
		hid_destroy_device(nano->hid[i]);
		nano->hid[i] = NULL;
	}
}

static int nanosic_register_hid(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	unsigned int i;
	int ret;

	if (nano->hid[0])
		return 0;
	for (i = 0; i < ARRAY_SIZE(nanosic_hid_info); i++) {
		const struct nanosic_hid_info *info = &nanosic_hid_info[i];
		struct hid_device *hid = hid_allocate_device();

		if (IS_ERR(hid)) {
			ret = PTR_ERR(hid);
			goto fail;
		}
		nano->hid[i] = hid;
		hid->ll_driver = &nanosic_hid_ll_driver;
		hid->dev.parent = dev;
		hid->bus = BUS_I2C;
		hid->vendor = NANOSIC_VENDOR_ID;
		hid->product = info->product;
		strscpy(hid->name, info->name, sizeof(hid->name));
		snprintf(hid->phys, sizeof(hid->phys), "%s/input%u", dev_name(dev), i);
		ret = hid_add_device(hid);
		if (ret)
			goto fail;
	}
	return 0;

fail:
	nanosic_destroy_hid(nano);
	return ret;
}

static int nanosic_handle_report(struct nanosic *nano, const struct nanosic_report *report)
{
	u8 *data = report->data;
	unsigned int i;

	if (data[0] == 0x23 && report->wire_size == 32 &&
	    data[2] == 0x38 && data[3] == 0x80 && data[4] == 0xa2) {
		u8 attach = data[9];

		nano->attach = attach;
		complete_all(&nano->attach_known);
		/*
		 * Only the keyboard tells laptop from tablet use: the folio
		 * hall sensors read the same for a cover folded back and for
		 * no cover at all. The input core drops unchanged states.
		 */
		if (nano->tablet_registered) {
			input_report_switch(nano->tablet, SW_TABLET_MODE,
					    !(attach & NANOSIC_ATTACH_CONNECTED));
			input_sync(nano->tablet);
		}
		if (!(attach & NANOSIC_ATTACH_CONNECTED))
			nanosic_destroy_hid(nano);
		else if ((attach & (NANOSIC_ATTACH_CONNECTED | NANOSIC_ATTACH_POWERED)) ==
			 (NANOSIC_ATTACH_CONNECTED | NANOSIC_ATTACH_POWERED))
			return nanosic_register_hid(nano);
		return 0;
	}
	if (!report->hid_size)
		return 0;
	for (i = 0; i < ARRAY_SIZE(nanosic_hid_info); i++)
		if (nanosic_hid_info[i].report_id == data[0] && nano->hid[i])
			return hid_input_report(nano->hid[i], HID_INPUT_REPORT,
						data, report->hid_size, 1);
	return 0;
}

/*
 * Allocated before the interrupt is requested, so that devres unregisters it
 * only after the interrupt is gone; registered once the attach state is known.
 */
static int nanosic_alloc_tablet(struct nanosic *nano)
{
	struct device *dev = &nano->client->dev;
	struct input_dev *input;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	input->name = "Xiaomi Keyboard Tablet Mode";
	input->phys = dev_name(dev);
	input->id.bustype = BUS_I2C;
	input->id.vendor = NANOSIC_VENDOR_ID;
	input_set_capability(input, EV_SW, SW_TABLET_MODE);
	nano->tablet = input;
	return 0;
}

static int nanosic_register_tablet(struct nanosic *nano)
{
	int ret;

	mutex_lock(&nano->lock);
	__assign_bit(SW_TABLET_MODE, nano->tablet->sw,
		     !(nano->attach & NANOSIC_ATTACH_CONNECTED));
	ret = input_register_device(nano->tablet);
	nano->tablet_registered = !ret;
	mutex_unlock(&nano->lock);
	return ret;
}

static irqreturn_t nanosic_irq(int irq, void *data)
{
	struct nanosic *nano = data;
	struct nanosic_report report;
	u8 frame[NANOSIC_FRAME_SIZE];
	size_t cursor = 0;
	int ret;

	mutex_lock(&nano->lock);
	ret = nanosic_read(nano, NANOSIC_REPORT_REGISTER, frame, sizeof(frame));
	if (ret)
		goto out;
	while ((ret = nanosic_frame_next(frame, sizeof(frame), &cursor, &report)) > 0) {
		ret = nanosic_handle_report(nano, &report);
		if (ret)
			break;
	}
out:
	mutex_unlock(&nano->lock);
	if (ret < 0)
		dev_err_ratelimited(&nano->client->dev, "input processing failed: %d\n", ret);
	return IRQ_HANDLED;
}

static void nanosic_power_off(void *data)
{
	struct nanosic *nano = data;

	if (!nano->powered)
		return;
	gpiod_set_value_cansleep(nano->wake_gpio, 0);
	gpiod_set_value_cansleep(nano->reset_gpio, 1);
	regulator_disable(nano->supplies[1].consumer);
	regulator_disable(nano->supplies[0].consumer);
	nano->powered = false;
}

static int nanosic_power_on(struct nanosic *nano)
{
	int ret;

	ret = regulator_enable(nano->supplies[0].consumer);
	if (ret)
		return ret;
	ret = regulator_enable(nano->supplies[1].consumer);
	if (ret) {
		regulator_disable(nano->supplies[0].consumer);
		return ret;
	}
	nano->powered = true;

	/* Preserve piano's stock reset/wake sequence, without recovery retries. */
	fsleep(2000);
	gpiod_set_value_cansleep(nano->reset_gpio, 0);
	msleep(500);
	gpiod_set_value_cansleep(nano->wake_gpio, 1);
	msleep(50);
	return 0;
}

static int nanosic_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct firmware *image;
	const char *fw_name = "nanosic/piano/MCU_Upgrade.bin";
	struct nanosic_firmware fw;
	struct nanosic *nano;
	int ret;

	if (client->addr != 0x4c || !i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "missing data interrupt\n");
	nano = devm_kzalloc(dev, sizeof(*nano), GFP_KERNEL);
	if (!nano)
		return -ENOMEM;
	nano->client = client;
	mutex_init(&nano->lock);
	init_completion(&nano->attach_known);
	i2c_set_clientdata(client, nano);
	device_property_read_string(dev, "firmware-name", &fw_name);
	ret = request_firmware(&image, fw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to load firmware\n");
	ret = nanosic_firmware_parse(image->data, image->size, &fw);
	if (!ret && get_unaligned_le16(fw.header + 25) != 0x1294)
		ret = -EINVAL;
	if (ret) {
		dev_err_probe(dev, ret, "unsupported piano firmware\n");
		goto out;
	}

	nano->supplies[0].supply = "vdd";
	nano->supplies[1].supply = "dvdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(nano->supplies), nano->supplies);
	if (ret)
		goto out;
	nano->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(nano->reset_gpio)) {
		ret = PTR_ERR(nano->reset_gpio);
		goto out;
	}
	nano->wake_gpio = devm_gpiod_get(dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(nano->wake_gpio)) {
		ret = PTR_ERR(nano->wake_gpio);
		goto out;
	}
	ret = nanosic_power_on(nano);
	if (ret)
		goto out;
	ret = devm_add_action_or_reset(dev, nanosic_power_off, nano);
	if (ret)
		goto out;
	ret = nanosic_boot_firmware(nano, &fw);
	if (ret) {
		dev_err_probe(dev, ret, "firmware bootstrap failed\n");
		goto out;
	}
	/* Stock waits 200 ms after starting the application, before input I/O. */
	msleep(200);
	ret = devm_add_action_or_reset(dev, nanosic_destroy_hid, nano);
	if (ret)
		goto out;
	ret = nanosic_alloc_tablet(nano);
	if (ret)
		goto out;
	ret = devm_request_threaded_irq(dev, client->irq, NULL, nanosic_irq,
					IRQF_ONESHOT, dev_name(dev), nano);
	if (ret)
		goto out;

	mutex_lock(&nano->lock);
	ret = nanosic_app_command(nano, 0x25, 1);
	if (!ret)
		ret = nanosic_app_command(nano, 0xa1, 1);
	mutex_unlock(&nano->lock);
	if (ret)
		goto out;

	/*
	 * 0xa1 asks for the attach state; the MCU answers with the 0xa2 report
	 * whether or not a keyboard is there. Register the switch with that
	 * state rather than flipping it right after registration.
	 */
	if (!wait_for_completion_timeout(&nano->attach_known,
					 msecs_to_jiffies(NANOSIC_ATTACH_TIMEOUT_MS)))
		dev_warn(dev, "no attach state from the MCU, assuming no keyboard\n");
	ret = nanosic_register_tablet(nano);
out:
	release_firmware(image);
	return ret;
}

static void nanosic_shutdown(struct i2c_client *client)
{
	struct nanosic *nano = i2c_get_clientdata(client);

	disable_irq(client->irq);
	nanosic_destroy_hid(nano);
	nanosic_power_off(nano);
}

static const struct of_device_id nanosic_of_match[] = {
	{ .compatible = "xiaomi,piano-keyboard" },
	{}
};
MODULE_DEVICE_TABLE(of, nanosic_of_match);

static struct i2c_driver nanosic_driver = {
	.driver = {
		.name = "hid-nanosic",
		.of_match_table = nanosic_of_match,
	},
	.probe = nanosic_probe,
	.shutdown = nanosic_shutdown,
};
module_i2c_driver(nanosic_driver);

#if IS_ENABLED(CONFIG_HID_NANOSIC_KUNIT_TEST)
#include "hid-nanosic-test.c"
#endif

MODULE_DESCRIPTION("Nanosic WN8030 transport for Xiaomi piano");
MODULE_FIRMWARE("nanosic/piano/MCU_Upgrade.bin");
MODULE_LICENSE("GPL");
