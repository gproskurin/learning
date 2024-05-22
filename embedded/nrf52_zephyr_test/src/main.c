#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define PRIO_LED 5

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
                return;
        }

        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        printk("Connected to \"%s\"\n", addr);

        if (bt_conn_set_security(conn, BT_SECURITY_L3)) {
                printk("connected: filed to set security\n");
        }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    printk("le_param_req\n");
    return true;
}

static void le_param_updated(struct bt_conn*, uint16_t interval, uint16_t latency, uint16_t timeout)
{
}

static void identity_resolved(struct bt_conn*, const bt_addr_le_t*, const bt_addr_le_t*)
{
}

static void conn_cb_security_changed(struct bt_conn*, bt_security_t level, enum bt_security_err err)
{
        printk("security changed: level=%i err=%i\n", (int)level, (int)err);
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
        .le_param_req = le_param_req,
        .le_param_updated = le_param_updated,
        .identity_resolved = identity_resolved,
        .security_changed = conn_cb_security_changed
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cb_passkey_display(struct bt_conn*, unsigned passkey)
{
        printk("passkey_display: PASSKEY=%06u\n", passkey);
}

static void auth_cb_auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static const struct bt_conn_auth_cb auth_cb = {
        .passkey_display = auth_cb_passkey_display,
	.cancel = auth_cb_auth_cancel
};

static void auth_info_cb_pairing_complete(struct bt_conn*, bool bonded)
{
        printk("auth_info: pairing_comnplete: bonded=%s\n", (bonded ? "yes":"no"));
}

static struct bt_conn_auth_info_cb auth_info_cb = {
        .pairing_complete = auth_info_cb_pairing_complete
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
	static uint8_t heartrate = 90U;
        static uint8_t delta = 1;
        if (heartrate > 160)
            delta = -1;
        else if (heartrate < 90)
            delta = 1;
	heartrate += delta;

	bt_hrs_notify(heartrate);
}

static const struct gpio_dt_spec dt_led = GPIO_DT_SPEC_GET(DT_ALIAS(my_led), gpios);
void thr_blink_fast(void*, void*, void*)
{
    if (!device_is_ready(dt_led.port)) {
        printk("GPIO device not ready\n");
        return;
    }
    if (gpio_pin_configure_dt(&dt_led, GPIO_OUTPUT_ACTIVE) < 0) {
        printk("Error configuring pin\n");
        return;
    }
    while(1) {
        gpio_pin_set_dt(&dt_led, 0);
        k_msleep(990);
        gpio_pin_set_dt(&dt_led, 1);
        k_msleep(10);
    }
}


K_THREAD_DEFINE(tid_blink_fast, 256, thr_blink_fast, NULL, NULL, NULL, PRIO_LED, 0, 0);

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_conn_auth_cb_register(&auth_cb);
	bt_conn_auth_info_cb_register(&auth_info_cb);
	bt_ready();

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	for (uint16_t i=0;; ++i) {
		k_sleep(K_SECONDS(5));

		hrs_notify();

                if ((i & 0xf) == 0) {
                        bas_notify();
                }
	}
	return 0;
}

