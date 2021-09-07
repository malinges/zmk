/*
 * Copyright (c) 2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <drivers/behavior.h>
#include <logging/log.h>
#include <zmk/behavior.h>

#define DT_DRV_COMPAT zmk_behavior_auto_repeat

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum behavior_auto_repeat_status {
    AR_STATUS_STOPPED,
    AR_STATUS_INITIAL_PRESS,
    AR_STATUS_PRESS,
    AR_STATUS_RELEASE,
};

struct behavior_auto_repeat_state {
    const struct behavior_auto_repeat_config *config;
    struct k_delayed_work work;
    struct zmk_behavior_binding_event event;
    enum behavior_auto_repeat_status status;
};

struct behavior_auto_repeat_config {
    struct behavior_auto_repeat_state *state;
    int initial_delay_ms;
    int repeat_delay_ms;
    int press_duration_ms;
    struct zmk_behavior_binding *behaviors;
    int behavior_count;
};

static void auto_repeat_press(const struct behavior_auto_repeat_config *cfg) {
    struct behavior_auto_repeat_state *state = cfg->state;

    for (int index = 0; index < cfg->behavior_count; index++) {
        const struct device *behavior = device_get_binding(cfg->behaviors[index].behavior_dev);
        if (!behavior) {
            break;
        }
        behavior_keymap_binding_pressed(&cfg->behaviors[index], state->event);
    }
}

static void auto_repeat_release(const struct behavior_auto_repeat_config *cfg) {
    struct behavior_auto_repeat_state *state = cfg->state;

    for (int index = 0; index < cfg->behavior_count; index++) {
        const struct device *behavior = device_get_binding(cfg->behaviors[index].behavior_dev);
        if (!behavior) {
            break;
        }
        behavior_keymap_binding_released(&cfg->behaviors[index], state->event);
    }
}

static void auto_repeat_work_handler(struct k_work *work) {
    struct behavior_auto_repeat_state *state =
        CONTAINER_OF(work, struct behavior_auto_repeat_state, work);
    const struct behavior_auto_repeat_config *cfg = state->config;

    int duration_ms = 0;

    switch (state->status) {
        case AR_STATUS_STOPPED:
            return;
        case AR_STATUS_INITIAL_PRESS:
            auto_repeat_release(cfg);
            state->status = AR_STATUS_RELEASE;
            duration_ms = cfg->initial_delay_ms - cfg->press_duration_ms;
            break;
        case AR_STATUS_PRESS:
            auto_repeat_release(cfg);
            state->status = AR_STATUS_RELEASE;
            duration_ms = cfg->repeat_delay_ms - cfg->press_duration_ms;
            break;
        case AR_STATUS_RELEASE:
            auto_repeat_press(cfg);
            state->status = AR_STATUS_PRESS;
            duration_ms = cfg->press_duration_ms;
            break;
    }

    int rc = k_delayed_work_submit(&state->work, K_MSEC(duration_ms));
    if (rc != 0) {
        LOG_ERR("auto_repeat: k_delayed_work_submit() returned %d", rc);
    }
}

static int behavior_auto_repeat_init(const struct device *dev) {
    const struct behavior_auto_repeat_config *cfg = dev->config;
    struct behavior_auto_repeat_state *state = cfg->state;

    state->config = cfg;
    k_delayed_work_init(&state->work, auto_repeat_work_handler);

    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    const struct behavior_auto_repeat_config *cfg = dev->config;
    struct behavior_auto_repeat_state *state = cfg->state;

    state->event = event;
    state->status = AR_STATUS_INITIAL_PRESS;
    auto_repeat_press(cfg);

    int rc = k_delayed_work_submit(&state->work, K_MSEC(cfg->press_duration_ms));
    if (rc != 0) {
        LOG_ERR("auto_repeat: k_delayed_work_submit() returned %d", rc);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    const struct behavior_auto_repeat_config *cfg = dev->config;
    struct behavior_auto_repeat_state *state = cfg->state;

    if (state->status == AR_STATUS_INITIAL_PRESS || state->status == AR_STATUS_PRESS) {
        auto_repeat_release(cfg);
    }

    state->status = AR_STATUS_STOPPED;

    int rc = k_delayed_work_cancel(&state->work);
    if (rc != 0) {
        LOG_ERR("auto_repeat: k_delayed_work_cancel() returned %d", rc);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_auto_repeat_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define _TRANSFORM_ENTRY(idx, node)                                                                \
    {                                                                                              \
        .behavior_dev = DT_LABEL(DT_INST_PHANDLE_BY_IDX(node, bindings, idx)),                     \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param1), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param1))),                  \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param2), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param2))),                  \
    },

#define TRANSFORMED_BINDINGS(node)                                                                 \
    { UTIL_LISTIFY(DT_INST_PROP_LEN(node, bindings), _TRANSFORM_ENTRY, node) }

#define KP_INST(n)                                                                                 \
    static struct zmk_behavior_binding                                                             \
        behavior_auto_repeat_config_##n##_bindings[DT_INST_PROP_LEN(n, bindings)] =                \
            TRANSFORMED_BINDINGS(n);                                                               \
    static struct behavior_auto_repeat_state behavior_auto_repeat_state_##n = {                    \
        .status = AR_STATUS_STOPPED,                                                               \
    };                                                                                             \
    static struct behavior_auto_repeat_config behavior_auto_repeat_config_##n = {                  \
        .state = &behavior_auto_repeat_state_##n,                                                  \
        .initial_delay_ms = DT_INST_PROP(n, initial_delay_ms),                                     \
        .repeat_delay_ms = DT_INST_PROP(n, repeat_delay_ms),                                       \
        .press_duration_ms = DT_INST_PROP(n, press_duration_ms),                                   \
        .behaviors = behavior_auto_repeat_config_##n##_bindings,                                   \
        .behavior_count = DT_INST_PROP_LEN(n, bindings),                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, behavior_auto_repeat_init, device_pm_control_nop, NULL,               \
                          &behavior_auto_repeat_config_##n, APPLICATION,                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_auto_repeat_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
