/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf_bellboard_local

#include <zephyr/devicetree.h>
#include <zephyr/drivers/mbox.h>

#include <haly/nrfy_bellboard.h>

#define BELLBOARD_IRQ_COUNT 4U

/* BELLBOARD runtime resources for each group */
struct mbox_bellboard_local_group {
	mbox_callback_t cb[NRF_BELLBOARD_EVENTS_TRIGGERED_COUNT];
	void *user_data[NRF_BELLBOARD_EVENTS_TRIGGERED_COUNT];
	uint32_t enabled_mask;
};

#define BELLBOARD_GRP_MAYBE_DEFINE(grp)                                                            \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(0, grp),                                                  \
		    (static struct mbox_bellboard_local_group g_##grp;), ())

#define BELLBOARD_GRP_GET_OR_NULL(grp) COND_CODE_1(DT_INST_IRQ_HAS_NAME(0, grp), (&g_##grp), (NULL))

BELLBOARD_GRP_MAYBE_DEFINE(bellboard0)
BELLBOARD_GRP_MAYBE_DEFINE(bellboard1)
BELLBOARD_GRP_MAYBE_DEFINE(bellboard2)
BELLBOARD_GRP_MAYBE_DEFINE(bellboard3)

static struct mbox_bellboard_local_group *const grps[BELLBOARD_IRQ_COUNT] = {
	BELLBOARD_GRP_GET_OR_NULL(bellboard0),
	BELLBOARD_GRP_GET_OR_NULL(bellboard1),
	BELLBOARD_GRP_GET_OR_NULL(bellboard2),
	BELLBOARD_GRP_GET_OR_NULL(bellboard3),
};

/* BELLBOARD IRQ list for each group */
#define GRP_IRQ_GET_OR_ZERO(grp)                                                                   \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(0, grp), (DT_INST_IRQ_BY_NAME(0, grp, irq)), (0U))

static const uint8_t irqs[BELLBOARD_IRQ_COUNT] = {
	GRP_IRQ_GET_OR_ZERO(bellboard0),
	GRP_IRQ_GET_OR_ZERO(bellboard1),
	GRP_IRQ_GET_OR_ZERO(bellboard2),
	GRP_IRQ_GET_OR_ZERO(bellboard3),
};

/* BELLBOARD IRQ mappings */
#define GET_IRQ_MAPPING_OR_ZERO(idx)                                                               \
	COND_CODE_1(DT_INST_PROP_HAS_IDX(0, nordic_interrupt_mapping, idx),                        \
		    (DT_INST_PROP_BY_IDX(0, nordic_interrupt_mapping, idx)), (0U))

static const uint32_t irq_mappings[BELLBOARD_IRQ_COUNT] = {
	GET_IRQ_MAPPING_OR_ZERO(0),
	GET_IRQ_MAPPING_OR_ZERO(2),
	GET_IRQ_MAPPING_OR_ZERO(4),
	GET_IRQ_MAPPING_OR_ZERO(6),
};

/* BELLBOARD instance */
static NRF_BELLBOARD_Type *bellboard = (NRF_BELLBOARD_Type *)DT_INST_REG_ADDR(0);

BUILD_ASSERT(DT_INST_PROP_LEN(0, nordic_interrupt_mapping) == 2 * DT_NUM_IRQS(DT_DRV_INST(0)),
	     "# interrupt mappings != # interrupts");

static void bellboard_local_isr(const void *parameter)
{
	uint8_t grp_idx = (uintptr_t)parameter;
	struct mbox_bellboard_local_group *grp = grps[grp_idx];
	uint32_t int_pend;

	int_pend = nrfy_bellboard_int_pending_get(bellboard, grp_idx);
	(void)nrfy_bellboard_events_process(bellboard, int_pend);

	for (uint8_t i = 0U; i < NRF_BELLBOARD_EVENTS_TRIGGERED_COUNT; i++) {
		if ((int_pend & BIT(i)) != 0U) {
			if (grp->cb[i] != NULL) {
				grp->cb[i](DEVICE_DT_INST_GET(0), i, grp->user_data[i], NULL);
			}
		}
	}
}

static int bellboard_local_grp_get(uint32_t id, uint8_t *grp)
{
	if (id >= NRF_BELLBOARD_EVENTS_TRIGGERED_COUNT) {
		return -EINVAL;
	}

	for (uint8_t i = 0U; i < ARRAY_SIZE(irq_mappings); i++) {
		if ((irq_mappings[i] & BIT(id)) != 0U) {
			*grp = i;
			return 0;
		}
	}

	return -EINVAL;
}

static uint32_t bellboard_local_max_channels_get(const struct device *dev)
{
	uint32_t max_channels = 0U;

	ARG_UNUSED(dev);

	for (uint8_t i = 0U; i < ARRAY_SIZE(grps); i++) {
		if (grps[i] != NULL) {
			max_channels += NRF_BELLBOARD_EVENTS_TRIGGERED_COUNT;
		}
	}

	return max_channels;
}

static int bellboard_local_register_callback(const struct device *dev, uint32_t id,
					     mbox_callback_t cb, void *user_data)
{
	struct mbox_bellboard_local_group *grp;
	uint8_t grp_idx;
	int ret;

	ARG_UNUSED(dev);

	ret = bellboard_local_grp_get(id, &grp_idx);
	if (ret < 0) {
		return ret;
	}

	grp = grps[grp_idx];

	grp->cb[id] = cb;
	grp->user_data[id] = user_data;

	return 0;
}

static int bellboard_local_set_enabled(const struct device *dev, uint32_t id, bool enable)
{
	struct mbox_bellboard_local_group *grp;
	uint8_t grp_idx;
	int ret;

	ARG_UNUSED(dev);

	ret = bellboard_local_grp_get(id, &grp_idx);
	if (ret < 0) {
		return ret;
	}

	grp = grps[grp_idx];

	if (enable) {
		uint32_t enabled_mask_prev;

		if ((grp->enabled_mask & BIT(id)) != 0U) {
			return -EALREADY;
		}

		enabled_mask_prev = grp->enabled_mask;
		grp->enabled_mask |= BIT(id);
		nrfy_bellboard_int_enable(bellboard, grp_idx, BIT(id));

		if (enabled_mask_prev == 0U) {
			irq_enable(irqs[grp_idx]);
		}
	} else {
		if ((grp->enabled_mask & BIT(id)) == 0U) {
			return -EALREADY;
		}

		grp->enabled_mask &= ~BIT(id);
		nrfy_bellboard_int_disable(bellboard, grp_idx, BIT(id));

		if (grp->enabled_mask == 0U) {
			irq_disable(irqs[grp_idx]);
		}
	}

	return 0;
}

static const struct mbox_driver_api bellboard_local_driver_api = {
	.max_channels_get = bellboard_local_max_channels_get,
	.register_callback = bellboard_local_register_callback,
	.set_enabled = bellboard_local_set_enabled,
};

#define BELLBOARD_IRQ_CONNECT(grp, grp_idx)                                                        \
	COND_CODE_1(DT_INST_IRQ_HAS_NAME(0, grp),                                                  \
		    (IRQ_CONNECT(DT_INST_IRQ_BY_NAME(0, grp, irq),                                 \
				 DT_INST_IRQ_BY_NAME(0, grp, priority), bellboard_local_isr,       \
				 (const void *)grp_idx, 0);),                                      \
		    ())

static int bellboard_local_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	for (uint8_t i = 0U; i < ARRAY_SIZE(grps); i++) {
		if (grps[i] != NULL) {
			nrfy_bellboard_int_init(bellboard, 0, 0, false, i);
		}
	}

	BELLBOARD_IRQ_CONNECT(bellboard0, 0);
	BELLBOARD_IRQ_CONNECT(bellboard1, 1);
	BELLBOARD_IRQ_CONNECT(bellboard2, 2);
	BELLBOARD_IRQ_CONNECT(bellboard3, 3);

	return 0;
}

DEVICE_DT_INST_DEFINE(0, bellboard_local_init, NULL, NULL, NULL, POST_KERNEL,
		      CONFIG_MBOX_INIT_PRIORITY, &bellboard_local_driver_api);
