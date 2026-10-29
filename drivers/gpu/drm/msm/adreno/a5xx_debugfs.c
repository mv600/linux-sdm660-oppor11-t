// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
 */

#include <linux/types.h>
#include <linux/debugfs.h>

#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>
#include <drm/drm_print.h>

#include "a5xx_gpu.h"

static void pfp_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	int i;

	drm_printf(p, "PFP state:\n");

	for (i = 0; i < 36; i++) {
		gpu_write(gpu, REG_A5XX_CP_PFP_STAT_ADDR, i);
		drm_printf(p, "  %02x: %08x\n", i,
			gpu_read(gpu, REG_A5XX_CP_PFP_STAT_DATA));
	}
}

static void me_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	int i;

	drm_printf(p, "ME state:\n");

	for (i = 0; i < 29; i++) {
		gpu_write(gpu, REG_A5XX_CP_ME_STAT_ADDR, i);
		drm_printf(p, "  %02x: %08x\n", i,
			gpu_read(gpu, REG_A5XX_CP_ME_STAT_DATA));
	}
}

static void meq_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	int i;

	drm_printf(p, "MEQ state:\n");
	gpu_write(gpu, REG_A5XX_CP_MEQ_DBG_ADDR, 0);

	for (i = 0; i < 64; i++) {
		drm_printf(p, "  %02x: %08x\n", i,
			gpu_read(gpu, REG_A5XX_CP_MEQ_DBG_DATA));
	}
}

static void roq_print(struct msm_gpu *gpu, struct drm_printer *p)
{
	int i;

	drm_printf(p, "ROQ state:\n");
	gpu_write(gpu, REG_A5XX_CP_ROQ_DBG_ADDR, 0);

	for (i = 0; i < 512 / 4; i++) {
		uint32_t val[4];
		int j;
		for (j = 0; j < 4; j++)
			val[j] = gpu_read(gpu, REG_A5XX_CP_ROQ_DBG_DATA);
		drm_printf(p, "  %02x: %08x %08x %08x %08x\n", i,
			val[0], val[1], val[2], val[3]);
	}
}

static int show(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_printer p = drm_seq_file_printer(m);
	void (*show)(struct msm_gpu *gpu, struct drm_printer *p) =
		node->info_ent->data;

	show(priv->gpu, &p);
	return 0;
}

#define ENT(n) { .name = #n, .show = show, .data = n ##_print }
static struct drm_info_list a5xx_debugfs_list[] = {
	ENT(pfp),
	ENT(me),
	ENT(meq),
	ENT(roq),
};

/* for debugfs files that can be written to, we can't use drm helper: */
static int
reset_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);

	if (!capable(CAP_SYS_ADMIN))
		return -EINVAL;

	/* TODO do we care about trying to make sure the GPU is idle?
	 * Since this is just a debug feature limited to CAP_SYS_ADMIN,
	 * maybe it is fine to let the user keep both pieces if they
	 * try to reset an active GPU.
	 */

	mutex_lock(&gpu->lock);

	release_firmware(adreno_gpu->fw[ADRENO_FW_PM4]);
	adreno_gpu->fw[ADRENO_FW_PM4] = NULL;

	release_firmware(adreno_gpu->fw[ADRENO_FW_PFP]);
	adreno_gpu->fw[ADRENO_FW_PFP] = NULL;

	if (a5xx_gpu->pm4_bo) {
		msm_gem_unpin_iova(a5xx_gpu->pm4_bo, gpu->vm);
		drm_gem_object_put(a5xx_gpu->pm4_bo);
		a5xx_gpu->pm4_bo = NULL;
	}

	if (a5xx_gpu->pfp_bo) {
		msm_gem_unpin_iova(a5xx_gpu->pfp_bo, gpu->vm);
		drm_gem_object_put(a5xx_gpu->pfp_bo);
		a5xx_gpu->pfp_bo = NULL;
	}

	gpu->needs_hw_init = true;

	pm_runtime_get_sync(&gpu->pdev->dev);
	gpu->funcs->recover(gpu);

	pm_runtime_put_sync(&gpu->pdev->dev);
	mutex_unlock(&gpu->lock);

	return 0;
}

static int speedbin_get(void *data, u64 *val)
{
	struct drm_device *dev = data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);

	if (val)
		*val = adreno_gpu->speedbin;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(reset_fops, NULL, reset_set, "%llx\n");
DEFINE_DEBUGFS_ATTRIBUTE(speedbin_fops, speedbin_get, NULL, "%llu\n");


/*
 * Temporary bring-up probe for the GPU debugfs reset.  Each write performs
 * exactly one stage and logs it before the MMIO access, so the last message
 * in pstore identifies the operation that stopped answering.
 */
static int gpu_probe_set(void *data, u64 val)
{
	struct drm_device *dev = data;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gpu *gpu = priv->gpu;
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	u32 v;
	int ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EINVAL;

	dev_info(gpu->dev->dev, "gpu-probe: stage %llu start\n", val);

	switch (val) {
	case 1:
		ret = mutex_lock_interruptible(&gpu->lock);
		if (ret)
			return ret;
		dev_info(gpu->dev->dev, "gpu-probe: lock acquired\n");
		ret = pm_runtime_get_sync(&gpu->pdev->dev);
		dev_info(gpu->dev->dev, "gpu-probe: runtime get returned %d\n", ret);
		pm_runtime_put_sync(&gpu->pdev->dev);
		mutex_unlock(&gpu->lock);
		return 0;
	case 2:
		ret = mutex_lock_interruptible(&gpu->lock);
		if (ret)
			return ret;
		pm_runtime_get_sync(&gpu->pdev->dev);
		dev_info(gpu->dev->dev, "gpu-probe: before hw_init\n");
		ret = msm_gpu_hw_init(gpu);
		dev_info(gpu->dev->dev, "gpu-probe: hw_init returned %d\n", ret);
		pm_runtime_put_sync(&gpu->pdev->dev);
		mutex_unlock(&gpu->lock);
		return 0;
	case 3:
		ret = mutex_lock_interruptible(&gpu->lock);
		if (ret)
			return ret;
		pm_runtime_get_sync(&gpu->pdev->dev);
		dev_info(gpu->dev->dev, "gpu-probe: before status3 read\n");
		v = gpu_read(gpu, REG_A5XX_RBBM_STATUS3);
		dev_info(gpu->dev->dev, "gpu-probe: status3=%08x\n", v);
		pm_runtime_put_sync(&gpu->pdev->dev);
		mutex_unlock(&gpu->lock);
		return 0;
	case 4:
		ret = mutex_lock_interruptible(&gpu->lock);
		if (ret)
			return ret;
		pm_runtime_get_sync(&gpu->pdev->dev);
		dev_info(gpu->dev->dev, "gpu-probe: before hwcg off\n");
		a5xx_set_hwcg(gpu, false);
		dev_info(gpu->dev->dev, "gpu-probe: hwcg off returned\n");
		a5xx_set_hwcg(gpu, true);
		dev_info(gpu->dev->dev, "gpu-probe: hwcg on returned\n");
		pm_runtime_put_sync(&gpu->pdev->dev);
		mutex_unlock(&gpu->lock);
		return 0;
	case 5 ... 28: {
		const unsigned int *regs;
		u32 start, end, addr;

		/* The generic register table has 128 ranges. */
		regs = to_adreno_gpu(gpu)->registers;
		ret = mutex_lock_interruptible(&gpu->lock);
		if (ret)
			return ret;
		pm_runtime_get_sync(&gpu->pdev->dev);
		start = (val - 5) * 8;
		end = min_t(u32, start + 7, 255);
		dev_info(gpu->dev->dev, "gpu-probe: register ranges %u..%u\n",
			 start, end);
		for (u32 i = start; i <= end; i += 2) {
			u32 first = regs[i];
			u32 last = regs[i + 1];

			dev_info(gpu->dev->dev, "gpu-probe: range %u: %04x..%04x\n",
				 i / 2, first, last);
			for (addr = first; addr <= last; addr++) {
				u32 v = gpu_read(gpu, addr);

				dev_info(gpu->dev->dev, "gpu-probe: %04x=%08x\n",
					 addr, v);
			}
		}
		dev_info(gpu->dev->dev, "gpu-probe: register group done\n");
		pm_runtime_put_sync(&gpu->pdev->dev);
		mutex_unlock(&gpu->lock);
		return 0;
	}
	default:
		return -EINVAL;
	}

	/* Keep the compiler happy about adreno_gpu when adding new stages. */
	(void)adreno_gpu;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gpu_probe_fops, NULL, gpu_probe_set, "%llu\n");


void a5xx_debugfs_init(struct msm_gpu *gpu, struct drm_minor *minor)
{
	struct drm_device *dev;

	if (!minor)
		return;

	dev = minor->dev;

	/* Guard against attempts to create duplicates  */
	if (!debugfs_lookup("reset", minor->debugfs_root)) {
		drm_debugfs_create_files(a5xx_debugfs_list,
					 ARRAY_SIZE(a5xx_debugfs_list),
					 minor->debugfs_root, minor);

		debugfs_create_file_unsafe("reset", S_IWUGO, minor->debugfs_root, dev,
					&reset_fops);
		debugfs_create_file_unsafe("speedbin", S_IRUGO, minor->debugfs_root, dev,
                                        &speedbin_fops);
		debugfs_create_file_unsafe("gpu-probe", S_IWUSR, minor->debugfs_root, dev,
					&gpu_probe_fops);
	}
}
