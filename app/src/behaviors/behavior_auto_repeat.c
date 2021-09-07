/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_behavior_auto_repeat

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ZMK_BHV_AUTO_REPEAT_MAX_HELD 10

struct behavior_auto_repeat_config {
    struct zmk_behavior_binding behavior;
    int initial_delay_ms;
    int repeat_delay_ms;
    int press_duration_ms;
};

enum auto_repeat_status {
    AR_STATUS_AVAILABLE,
    AR_STATUS_INITIAL_PRESS,
    AR_STATUS_REPEAT_PRESS,
    AR_STATUS_RELEASE,
    AR_STATUS_CANCELLED,
};

struct active_auto_repeat {
    const struct behavior_auto_repeat_config *config;
    int layer;
    uint32_t position;
    uint32_t param1;
    uint32_t param2;
    enum auto_repeat_status status;
    struct k_delayed_work work;
};

struct active_auto_repeat active_auto_repeats[ZMK_BHV_AUTO_REPEAT_MAX_HELD] = {};

static struct active_auto_repeat *find_available_auto_repeat(void) {
    for (int i = 0; i < ZMK_BHV_AUTO_REPEAT_MAX_HELD; i++) {
        struct active_auto_repeat *auto_repeat = &active_auto_repeats[i];
        if (auto_repeat->status == AR_STATUS_AVAILABLE) {
            return auto_repeat;
        }
    }
    return NULL;
}

static struct active_auto_repeat *find_active_auto_repeat(uint32_t position) {
    for (int i = 0; i < ZMK_BHV_AUTO_REPEAT_MAX_HELD; i++) {
        struct active_auto_repeat *auto_repeat = &active_auto_repeats[i];
        if (auto_repeat->status != AR_STATUS_AVAILABLE &&
            auto_repeat->status != AR_STATUS_CANCELLED &&
            auto_repeat->position == position) {
            return auto_repeat;
        }
    }
    return NULL;
}

static int auto_repeat_press(struct active_auto_repeat *auto_repeat, int64_t timestamp) {
    struct zmk_behavior_binding binding = {
        .behavior_dev = auto_repeat->config->behavior.behavior_dev,
        .param1 = auto_repeat->param1,
        .param2 = auto_repeat->param2,
    };

    struct zmk_behavior_binding_event event = {
        .layer = auto_repeat->layer,
        .position = auto_repeat->position,
        .timestamp = timestamp,
    };

    return behavior_keymap_binding_pressed(&binding, event);
}

static int auto_repeat_release(struct active_auto_repeat *auto_repeat, int64_t timestamp) {
    struct zmk_behavior_binding binding = {
        .behavior_dev = auto_repeat->config->behavior.behavior_dev,
        .param1 = auto_repeat->param1,
        .param2 = auto_repeat->param2,
    };

    struct zmk_behavior_binding_event event = {
        .layer = auto_repeat->layer,
        .position = auto_repeat->position,
        .timestamp = timestamp,
    };

    return behavior_keymap_binding_released(&binding, event);
}

static void clear_auto_repeat(struct active_auto_repeat *auto_repeat, int64_t timestamp) {
    if (auto_repeat->status != AR_STATUS_AVAILABLE && auto_repeat->status != AR_STATUS_CANCELLED) {
        if (auto_repeat->status == AR_STATUS_INITIAL_PRESS ||
            auto_repeat->status == AR_STATUS_REPEAT_PRESS) {
            auto_repeat_release(auto_repeat, timestamp);
        }

        int rc = k_delayed_work_cancel(&auto_repeat->work);
        if (rc == -EINVAL) {
            auto_repeat->status = AR_STATUS_CANCELLED;
        } else {
            auto_repeat->status = AR_STATUS_AVAILABLE;
        }
    }
}

static void auto_repeat_work_handler(struct k_work *work) {
    struct active_auto_repeat *auto_repeat =
        CONTAINER_OF(work, struct active_auto_repeat, work);
    const struct behavior_auto_repeat_config *cfg = auto_repeat->config;

    int duration_ms = 0;

    switch (auto_repeat->status) {
        case AR_STATUS_AVAILABLE:
            return;
        case AR_STATUS_INITIAL_PRESS:
            auto_repeat_release(auto_repeat, k_uptime_get());
            auto_repeat->status = AR_STATUS_RELEASE;
            duration_ms = cfg->initial_delay_ms - cfg->press_duration_ms;
            break;
        case AR_STATUS_REPEAT_PRESS:
            auto_repeat_release(auto_repeat, k_uptime_get());
            auto_repeat->status = AR_STATUS_RELEASE;
            duration_ms = cfg->repeat_delay_ms - cfg->press_duration_ms;
            break;
        case AR_STATUS_RELEASE:
            auto_repeat_press(auto_repeat, k_uptime_get());
            auto_repeat->status = AR_STATUS_REPEAT_PRESS;
            duration_ms = cfg->press_duration_ms;
            break;
        case AR_STATUS_CANCELLED:
            auto_repeat->status = AR_STATUS_AVAILABLE;
            return;
    }

    int rc = k_delayed_work_submit(&auto_repeat->work, K_MSEC(duration_ms));
    if (rc != 0) {
        LOG_ERR("auto_repeat: k_delayed_work_submit() returned %d", rc);
    }
}

static int behavior_auto_repeat_init(const struct device *dev) {
    static bool init_first_run = true;

    if (init_first_run) {
        for (int i = 0; i < ZMK_BHV_AUTO_REPEAT_MAX_HELD; i++) {
            struct active_auto_repeat *auto_repeat = &active_auto_repeats[i];
            k_delayed_work_init(&auto_repeat->work, auto_repeat_work_handler);
        }
        init_first_run = false;
    }

    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    const struct behavior_auto_repeat_config *cfg = dev->config;
    struct active_auto_repeat *auto_repeat;

    auto_repeat = find_active_auto_repeat(event.position);
    if (auto_repeat != NULL) {
        clear_auto_repeat(auto_repeat, event.timestamp);
    }

    auto_repeat = find_available_auto_repeat();
    if (auto_repeat != NULL) {
        auto_repeat->config = cfg;
        auto_repeat->layer = event.layer;
        auto_repeat->position = event.position;
        auto_repeat->param1 = binding->param1;
        auto_repeat->param2 = binding->param2;
        auto_repeat->status = AR_STATUS_INITIAL_PRESS;

        auto_repeat_press(auto_repeat, event.timestamp);

        int rc = k_delayed_work_submit(&auto_repeat->work, K_MSEC(cfg->press_duration_ms));
        if (rc != 0) {
            LOG_ERR("auto_repeat: k_delayed_work_submit() returned %d", rc);
        }
    } else {
        LOG_ERR("unable to store auto repeat, did you press more than %d auto_repeat?",
                ZMK_BHV_AUTO_REPEAT_MAX_HELD);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    struct active_auto_repeat *auto_repeat = find_active_auto_repeat(event.position);

    if (auto_repeat != NULL) {
        clear_auto_repeat(auto_repeat, event.timestamp);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_auto_repeat_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define KP_INST(n)                                                                                 \
    static struct behavior_auto_repeat_config behavior_auto_repeat_config_##n = {                  \
        .behavior = ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),                                 \
        .initial_delay_ms = DT_INST_PROP(n, initial_delay_ms),                                     \
        .repeat_delay_ms = DT_INST_PROP(n, repeat_delay_ms),                                       \
        .press_duration_ms = DT_INST_PROP(n, press_duration_ms),                                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, behavior_auto_repeat_init, device_pm_control_nop, NULL,               \
                          &behavior_auto_repeat_config_##n, APPLICATION,                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_auto_repeat_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
