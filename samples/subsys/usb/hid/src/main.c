/*
 * Copyright (c) 2016-2018 Intel Corporation.
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <inttypes.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

/////////////////////////////////////// USB SETUP
#define REPORT_ID_1	0x01
#define REPORT_ID_2	0x02

static struct k_delayed_work delayed_report_send;

static struct device *hdev;

#define REPORT_TIMEOUT K_SECONDS(2)

/* Some HID sample Report Descriptor */
static const u8_t hid_report_desc[] = {
	/* 0x05, 0x01,		USAGE_PAGE (Generic Desktop)		*/
	HID_GI_USAGE_PAGE, USAGE_GEN_DESKTOP,
	/* 0x09, 0x00,		USAGE (Undefined)			*/
	HID_LI_USAGE, USAGE_GEN_DESKTOP_KEYBOARD,
	/* 0xa1, 0x01,		COLLECTION (Application)		*/
	HID_MI_COLLECTION, COLLECTION_APPLICATION,
	/* 0x15, 0x00,			LOGICAL_MINIMUM one-byte (0)	*/
	HID_GI_LOGICAL_MIN(1), 0x00,
	/* 0x26, 0xff, 0x00,		LOGICAL_MAXIMUM two-bytes (255)	*/
	HID_GI_LOGICAL_MAX(2), 0xFF, 0x00,
	/* 0x85, 0x01,			REPORT_ID (1)			*/
	HID_GI_REPORT_ID, REPORT_ID_1,
	/* 0x75, 0x08,			REPORT_SIZE (8) in bits		*/
	HID_GI_REPORT_SIZE, 0x08,
	/* 0x95, 0x01,			REPORT_COUNT (1)		*/
	HID_GI_REPORT_COUNT, 0x01,
	/* 0x09, 0x00,			USAGE (Undefined)		*/
	HID_LI_USAGE, USAGE_GEN_DESKTOP_KEYBOARD,
	/* v0x81, 0x82,			INPUT (Data,Var,Abs,Vol)	*/
	HID_MI_INPUT, 0x82,
	/* 0x85, 0x02,			REPORT_ID (2)			*/
	HID_GI_REPORT_ID, REPORT_ID_2,
	/* 0x75, 0x08,			REPORT_SIZE (8) in bits		*/
	HID_GI_REPORT_SIZE, 0x08,
	/* 0x95, 0x01,			REPORT_COUNT (1)		*/
	HID_GI_REPORT_COUNT, 0x01,
	/* 0x09, 0x00,			USAGE (Undefined)		*/
	HID_LI_USAGE, USAGE_GEN_DESKTOP_UNDEFINED,
	/* 0x91, 0x82,			OUTPUT (Data,Var,Abs,Vol)	*/
	HID_MI_OUTPUT, 0x82,
	/* 0xc0			END_COLLECTION			*/
	HID_MI_COLLECTION_END,
};

static const u8_t my_HID_KEYBOARD_REPORT_DESC[] = HID_KEYBOARD_REPORT_DESC();

static void my_send_report(u8_t modifier, u8_t data[6]) {
                          // MOD,  RESV, DATA1,  DATA2, DATA3, DATA4, DATA5, DATA6
        u8_t report_1[8] = { modifier, 0x00, data[0], data[1], data[2], data[3], data[4], data[5] };
        int ret, wrote;

	ret = hid_int_ep_write(hdev, report_1, sizeof(report_1), &wrote);

	LOG_DBG("Wrote %d bytes with ret %d", wrote, ret);

}

static void in_ready_cb(void)
{
	k_delayed_work_submit(&delayed_report_send, REPORT_TIMEOUT);
}

static void status_cb(enum usb_dc_status_code status, const u8_t *param)
{
	switch (status) {
	case USB_DC_CONFIGURED:
		in_ready_cb();
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static void idle_cb(u16_t report_id)
{
	static u8_t report_1[2] = { 0x00, 0xEB };
	int ret, wrote;

	ret = hid_int_ep_write(hdev, report_1, sizeof(report_1), &wrote);

	LOG_DBG("Idle callback: wrote %d bytes with ret %d", wrote, ret);
}

static void protocol_cb(u8_t protocol)
{
	LOG_DBG("New protocol: %s", protocol == HID_PROTOCOL_BOOT ?
		"boot" : "report");
}

static const struct hid_ops ops = {
	.int_in_ready = in_ready_cb,
	.on_idle = idle_cb,
	.protocol_change = protocol_cb,
};

///////////////////////////////////////////////////////////////////////////// END USB SETUP

///////////////////////////////////////////////////////////////////////////// DIGITAL INPUT SETUP
bool volatile button1_pressed = false;
bool volatile buttons_update = false;

/*
 * Devicetree helper macro which gets the 'flags' cell from a 'gpios'
 * property, or returns 0 if the property has no 'flags' cell.
 */

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))

/*
 * Get button configuration from the devicetree sw0 alias.
 *
 * At least a GPIO device and pin number must be provided. The 'flags'
 * cell is optional.
 */

#define SW0_NODE	DT_ALIAS(sw0)

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
#define SW0_GPIO_LABEL	DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_GPIO_PIN	DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_GPIO_FLAGS	(GPIO_INPUT | FLAGS_OR_ZERO(SW0_NODE))
#else
#error "Unsupported board: sw0 devicetree alias is not defined"
#define SW0_GPIO_LABEL	""
#define SW0_GPIO_PIN	0
#define SW0_GPIO_FLAGS	0
#endif

/* LED helpers, which use the led0 devicetree alias if it's available. */
static struct device *initialize_led(void);
static void match_led_to_button(struct device *button, struct device *led);

static struct gpio_callback button_cb_data;

void button_pressed(struct device *dev, struct gpio_callback *cb,
		    u32_t pins)
{
        //printk("Button number pressed %" u32_t, pins);
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
        button1_pressed = ~ button1_pressed;
        buttons_update = true;
}

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */

#define LED0_NODE	DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay) && DT_NODE_HAS_PROP(LED0_NODE, gpios)
#define LED0_GPIO_LABEL	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_GPIO_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_GPIO_FLAGS	(GPIO_OUTPUT | FLAGS_OR_ZERO(LED0_NODE))
#endif

#ifdef LED0_GPIO_LABEL
static struct device *initialize_led(void)
{
	struct device *led;
	int ret;

	led = device_get_binding(LED0_GPIO_LABEL);
	if (led == NULL) {
		printk("Didn't find LED device %s\n", LED0_GPIO_LABEL);
		return NULL;
	}

	ret = gpio_pin_configure(led, LED0_GPIO_PIN, LED0_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure LED device %s pin %d\n",
		       ret, LED0_GPIO_LABEL, LED0_GPIO_PIN);
		return NULL;
	}

	printk("Set up LED at %s pin %d\n", LED0_GPIO_LABEL, LED0_GPIO_PIN);

	return led;
}

static void match_led_to_button(struct device *button, struct device *led)
{
	bool val;

	val = gpio_pin_get(button, SW0_GPIO_PIN);
	gpio_pin_set(led, LED0_GPIO_PIN, val);
}

#else  /* !defined(LED0_GPIO_LABEL) */
static struct device *initialize_led(void)
{
	printk("No LED device was defined\n");
	return NULL;
}

static void match_led_to_button(struct device *button, struct device *led)
{
	return;
}
#endif	/* LED0_GPIO_LABEL */

///////////////////////////////////////////////////////////////////////////// END DIGITAL INPUT SETUP

void main(void)
{
	int ret;

	LOG_DBG("Starting application");

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	k_delayed_work_init(&delayed_report_send, my_send_report);

        //// button stuff

        struct device *button;
	struct device *led;

	button = device_get_binding(SW0_GPIO_LABEL);
	if (button == NULL) {
		printk("Error: didn't find %s device\n", SW0_GPIO_LABEL);
		return;
	}

	ret = gpio_pin_configure(button, SW0_GPIO_PIN, SW0_GPIO_FLAGS);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	ret = gpio_pin_interrupt_configure(button,
					   SW0_GPIO_PIN,
					   GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, SW0_GPIO_LABEL, SW0_GPIO_PIN);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_GPIO_PIN));
	gpio_add_callback(button, &button_cb_data);
	printk("Set up button at %s pin %d\n", SW0_GPIO_LABEL, SW0_GPIO_PIN);

	led = initialize_led();

        //// end button stuff




        // u8_t a = REPORT_ID_1;
        u8_t counter = 0x04; // 
        u8_t nodata[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        u8_t data[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        u8_t modifier = 0x00;
        match_led_to_button(button, led);
        while(true) {
          if(buttons_update == true) {
            LOG_DBG("inside buttons update");
            match_led_to_button(button, led);
            k_msleep(1);
            buttons_update = false;
            
            if(button1_pressed) {
              // my_send_report(u8_t modifier, u8_t[6] data)
              my_send_report(modifier, data);
              
              counter++;
              if(counter > 0x27) {
                counter = 0x04;
              }

              data[0] = counter;
              k_msleep(1);
            }
          }
        }

}

static int composite_pre_init(struct device *dev)
{
	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return -ENODEV;
	}

	LOG_DBG("HID Device: dev %p", hdev);

	usb_hid_register_device(hdev, my_HID_KEYBOARD_REPORT_DESC, sizeof(my_HID_KEYBOARD_REPORT_DESC),
				&ops);

	return usb_hid_init(hdev);
}

SYS_INIT(composite_pre_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
