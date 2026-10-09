// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/input/mt.h>

struct nanosic_test_bus {
	struct nanosic nano;
	struct i2c_adapter adapter;
	int forced_result;
	u8 boot_id;
	u8 state;
	u8 header_state;
	u8 verified_state;
	u8 last_command;
	u8 last_register;
	u16 last_size;
	u32 transfers;
	u32 headers;
	u32 chunks;
	bool started;
	u8 frame[NANOSIC_FRAME_SIZE];
};

static int nanosic_test_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct nanosic_test_bus *bus = i2c_get_adapdata(adapter);
	u8 *data = msgs[0].buf;

	bus->transfers++;
	if (bus->forced_result != INT_MAX)
		return bus->forced_result;
	if (msgs[0].addr != 0x4c || !msgs[0].len)
		return -EINVAL;
	bus->last_register = data[0];
	bus->last_size = msgs[0].len;
	if (num == 2) {
		if (msgs[0].len != 1 || msgs[1].addr != 0x4c ||
		    msgs[1].flags != I2C_M_RD)
			return -EINVAL;
		memset(msgs[1].buf, 0, msgs[1].len);
		if (data[0] == NANOSIC_BOOT_REGISTER && msgs[1].len == 1)
			msgs[1].buf[0] = bus->last_command == 0x04 ? bus->boot_id : bus->state;
		else if (data[0] == NANOSIC_REPORT_REGISTER && msgs[1].len == sizeof(bus->frame))
			memcpy(msgs[1].buf, bus->frame, sizeof(bus->frame));
		return num;
	}
	if (num != 1 || msgs[0].len < 2)
		return -EINVAL;
	if (data[0] != NANOSIC_BOOT_REGISTER)
		return num;
	bus->last_command = data[1];
	switch (data[1]) {
	case 0x06:
		if (msgs[0].len != NANOSIC_FW_HEADER_CMD_SIZE + 1)
			return -EINVAL;
		bus->headers++;
		bus->state = bus->header_state;
		break;
	case 0x07:
		if (msgs[0].len != NANOSIC_FW_DATA_CMD_SIZE + 1)
			return -EINVAL;
		bus->chunks++;
		break;
	case 0x08:
		bus->state = bus->verified_state;
		break;
	case 0x09:
		bus->started = true;
		break;
	}
	return num;
}

static u32 nanosic_test_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm nanosic_test_algorithm = {
	.xfer = nanosic_test_xfer,
	.functionality = nanosic_test_functionality,
};

static void nanosic_test_del_adapter(void *data)
{
	i2c_del_adapter(data);
}

static void nanosic_test_del_client(void *data)
{
	i2c_unregister_device(data);
}

static int nanosic_test_init(struct kunit *test)
{
	struct nanosic_test_bus *bus;
	struct device *dev;
	int ret;

	bus = kunit_kzalloc(test, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	test->priv = bus;
	bus->forced_result = INT_MAX;
	bus->boot_id = 0xc8;
	bus->header_state = 1;
	bus->verified_state = 3;
	mutex_init(&bus->nano.lock);
	init_completion(&bus->nano.attach_known);
	dev = kunit_device_register(test, "nanosic-transport-test");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	bus->adapter.owner = THIS_MODULE;
	bus->adapter.dev.parent = dev;
	bus->adapter.algo = &nanosic_test_algorithm;
	strscpy(bus->adapter.name, "nanosic-test-bus", sizeof(bus->adapter.name));
	i2c_set_adapdata(&bus->adapter, bus);
	ret = i2c_add_adapter(&bus->adapter);
	if (ret)
		return ret;
	ret = kunit_add_action_or_reset(test, nanosic_test_del_adapter, &bus->adapter);
	if (ret)
		return ret;
	bus->nano.client = i2c_new_dummy_device(&bus->adapter, 0x4c);
	if (IS_ERR(bus->nano.client))
		return PTR_ERR(bus->nano.client);
	ret = kunit_add_action_or_reset(test, nanosic_test_del_client, bus->nano.client);
	if (ret)
		return ret;
	return kunit_add_action_or_reset(test, nanosic_destroy_hid, &bus->nano);
}

static void nanosic_test_transfer_errors(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;
	u8 data[NANOSIC_FRAME_SIZE] = {};

	bus->forced_result = 0;
	KUNIT_EXPECT_EQ(test, nanosic_write(&bus->nano, NANOSIC_REPORT_REGISTER, data, 1),
			-EIO);
	KUNIT_EXPECT_EQ(test, bus->transfers, 1U);
	bus->forced_result = 1;
	KUNIT_EXPECT_EQ(test, nanosic_read(&bus->nano, NANOSIC_REPORT_REGISTER,
					 data, sizeof(data)), -EIO);
	KUNIT_EXPECT_EQ(test, bus->transfers, 2U);
	bus->forced_result = -EREMOTEIO;
	KUNIT_EXPECT_EQ(test, nanosic_read(&bus->nano, NANOSIC_REPORT_REGISTER,
					 data, sizeof(data)), -EREMOTEIO);
	KUNIT_EXPECT_EQ(test, bus->transfers, 3U);
	bus->forced_result = INT_MAX;
	KUNIT_ASSERT_EQ(test, nanosic_app_command(&bus->nano, 0x25, 1), 0);
	KUNIT_EXPECT_EQ(test, bus->last_register, 0x4c);
	KUNIT_EXPECT_EQ(test, bus->last_size, 67);
}

/* The real stock image is covered by nanosic-protocol; this tests I/O control flow. */
static const u8 nanosic_test_header[64] = { 'N', 'A', 'N', 'O', ' ', 'I', 'C' };
static const u8 nanosic_test_code[4] = {};
static const struct nanosic_firmware nanosic_test_fw = {
	.header = nanosic_test_header,
	.code = nanosic_test_code,
	.size = sizeof(nanosic_test_code),
};

static void nanosic_test_boot_success(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;

	KUNIT_ASSERT_EQ(test, nanosic_boot_firmware(&bus->nano, &nanosic_test_fw), 0);
	KUNIT_EXPECT_TRUE(test, bus->started);
	KUNIT_EXPECT_EQ(test, bus->headers, 1U);
	KUNIT_EXPECT_EQ(test, bus->chunks, 1U);
	KUNIT_EXPECT_EQ(test, bus->last_register, 0x5c);
}

static void nanosic_test_boot_rejects_id(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;

	bus->boot_id = 0;
	KUNIT_EXPECT_EQ(test, nanosic_boot_firmware(&bus->nano, &nanosic_test_fw), -ENODEV);
	KUNIT_EXPECT_EQ(test, bus->headers, 0U);
	KUNIT_EXPECT_FALSE(test, bus->started);
}

static void nanosic_test_boot_rejects_state(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;

	bus->header_state = 2;
	KUNIT_EXPECT_EQ(test, nanosic_boot_firmware(&bus->nano, &nanosic_test_fw), -EPROTO);
	KUNIT_EXPECT_EQ(test, bus->chunks, 0U);
	KUNIT_EXPECT_FALSE(test, bus->started);
}

static void nanosic_test_boot_rejects_verification(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;

	bus->verified_state = 2;
	KUNIT_EXPECT_EQ(test, nanosic_boot_firmware(&bus->nano, &nanosic_test_fw), -EPROTO);
	KUNIT_EXPECT_EQ(test, bus->chunks, 1U);
	KUNIT_EXPECT_FALSE(test, bus->started);
}

static void nanosic_test_hid_attachment(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;
	u8 data[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2 };
	struct nanosic_report report = { .data = data, .wire_size = sizeof(data) };
	struct hid_device *keyboard;
	unsigned int i;

	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	KUNIT_EXPECT_PTR_EQ(test, bus->nano.hid[0], NULL);
	data[9] = 3;
	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	for (i = 0; i < NANOSIC_HID_COUNT; i++) {
		KUNIT_ASSERT_NOT_NULL(test, bus->nano.hid[i]);
		KUNIT_EXPECT_TRUE(test, bus->nano.hid[i]->claimed & HID_CLAIMED_INPUT);
	}
	KUNIT_EXPECT_EQ(test, bus->nano.hid[3]->group, HID_GROUP_MULTITOUCH_WIN_8);
	KUNIT_ASSERT_NOT_NULL(test, bus->nano.hid[3]->driver);
	KUNIT_EXPECT_STREQ(test, bus->nano.hid[3]->driver->name, "hid-multitouch");
	keyboard = bus->nano.hid[0];
	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	KUNIT_EXPECT_PTR_EQ(test, bus->nano.hid[0], keyboard);
	data[9] = 0;
	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	for (i = 0; i < NANOSIC_HID_COUNT; i++)
		KUNIT_EXPECT_PTR_EQ(test, bus->nano.hid[i], NULL);
}

static void nanosic_test_tablet_mode(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;
	u8 data[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2 };
	struct nanosic_report report = { .data = data, .wire_size = sizeof(data) };

	KUNIT_ASSERT_EQ(test, nanosic_alloc_tablet(&bus->nano), 0);
	KUNIT_EXPECT_FALSE(test, completion_done(&bus->nano.attach_known));
	/* Reports before registration only record the state. */
	data[9] = 0x23;
	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	KUNIT_EXPECT_TRUE(test, completion_done(&bus->nano.attach_known));
	KUNIT_ASSERT_EQ(test, nanosic_register_tablet(&bus->nano), 0);
	KUNIT_EXPECT_FALSE(test, test_bit(SW_TABLET_MODE, bus->nano.tablet->sw));
	data[9] = 0;
	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	KUNIT_EXPECT_TRUE(test, test_bit(SW_TABLET_MODE, bus->nano.tablet->sw));
	/* Connected but not yet powered is already laptop mode. */
	data[9] = 1;
	KUNIT_ASSERT_EQ(test, nanosic_handle_report(&bus->nano, &report), 0);
	KUNIT_EXPECT_FALSE(test, test_bit(SW_TABLET_MODE, bus->nano.tablet->sw));
	KUNIT_EXPECT_PTR_EQ(test, bus->nano.hid[0], NULL);
}

static void nanosic_test_tablet_mode_without_keyboard(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;

	/* No answer before the timeout: start in tablet mode. */
	KUNIT_ASSERT_EQ(test, nanosic_alloc_tablet(&bus->nano), 0);
	KUNIT_ASSERT_EQ(test, nanosic_register_tablet(&bus->nano), 0);
	KUNIT_EXPECT_TRUE(test, test_bit(SW_TABLET_MODE, bus->nano.tablet->sw));
}

static void nanosic_test_key_events(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;
	struct hid_input *input;

	KUNIT_ASSERT_EQ(test, nanosic_register_hid(&bus->nano), 0);
	KUNIT_ASSERT_FALSE(test, list_empty(&bus->nano.hid[0]->inputs));
	input = list_first_entry(&bus->nano.hid[0]->inputs, struct hid_input, list);
	bus->frame[0] = 0x57;
	bus->frame[2] = 0x39;
	bus->frame[3] = 0x05;
	bus->frame[6] = 0x04; /* Keyboard usage A. */
	KUNIT_ASSERT_EQ(test, nanosic_irq(0, &bus->nano), IRQ_HANDLED);
	KUNIT_EXPECT_TRUE(test, test_bit(KEY_A, input->input->key));
	bus->frame[6] = 0;
	KUNIT_ASSERT_EQ(test, nanosic_irq(0, &bus->nano), IRQ_HANDLED);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_A, input->input->key));
}

static void nanosic_test_touch_events(struct kunit *test)
{
	struct nanosic_test_bus *bus = test->priv;
	struct hid_input *input;
	struct input_mt *mt;
	u8 *report = bus->frame + 3;

	KUNIT_ASSERT_EQ(test, nanosic_register_hid(&bus->nano), 0);
	KUNIT_ASSERT_FALSE(test, list_empty(&bus->nano.hid[3]->inputs));
	input = list_first_entry(&bus->nano.hid[3]->inputs, struct hid_input, list);
	mt = input->input->mt;
	KUNIT_ASSERT_NOT_NULL(test, mt);
	bus->frame[0] = 0x57;
	bus->frame[2] = 0x39;
	report[0] = 0x19;
	report[2] = 7; /* Tip, in-range and confidence. */
	put_unaligned_le16(1000, report + 4);
	put_unaligned_le16(1200, report + 6);
	put_unaligned_le16(800, report + 8);
	report[26] = 1;
	KUNIT_ASSERT_EQ(test, nanosic_irq(0, &bus->nano), IRQ_HANDLED);
	KUNIT_EXPECT_TRUE(test, input_mt_is_active(&mt->slots[0]));
	KUNIT_EXPECT_EQ(test, input_mt_get_value(&mt->slots[0], ABS_MT_POSITION_X), 1200);
	KUNIT_EXPECT_EQ(test, input_mt_get_value(&mt->slots[0], ABS_MT_POSITION_Y), 800);
	KUNIT_EXPECT_EQ(test, input_mt_get_value(&mt->slots[0], ABS_MT_PRESSURE), 1000);
	report[2] = 0;
	KUNIT_ASSERT_EQ(test, nanosic_irq(0, &bus->nano), IRQ_HANDLED);
	KUNIT_EXPECT_FALSE(test, input_mt_is_active(&mt->slots[0]));
}

static struct kunit_case nanosic_transport_cases[] = {
	KUNIT_CASE(nanosic_test_transfer_errors),
	KUNIT_CASE(nanosic_test_boot_success),
	KUNIT_CASE(nanosic_test_boot_rejects_id),
	KUNIT_CASE(nanosic_test_boot_rejects_state),
	KUNIT_CASE(nanosic_test_boot_rejects_verification),
	KUNIT_CASE(nanosic_test_hid_attachment),
	KUNIT_CASE(nanosic_test_tablet_mode),
	KUNIT_CASE(nanosic_test_tablet_mode_without_keyboard),
	KUNIT_CASE(nanosic_test_key_events),
	KUNIT_CASE(nanosic_test_touch_events),
	{}
};

static struct kunit_suite nanosic_transport_suite = {
	.name = "nanosic-transport",
	.init = nanosic_test_init,
	.test_cases = nanosic_transport_cases,
};
kunit_test_suite(nanosic_transport_suite);
