/*
 * Vivante GPU Acceleration Xorg driver
 *
 * Written by Russell King, 2015
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_DIX_CONFIG_H
#include "dix-config.h"
#endif

#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

/* drm includes */
#include <xf86drm.h>

#include "fb.h"
#include "gcstruct.h"
#include "xf86.h"
#include "dri3.h"
#include "misyncshm.h"
#include "compat-api.h"

#include "etnaviv_drmif.h"
#include "etnaviv_drm.h"

#include "viv2d.h"
#include "viv2d_exa.h"

static Bool Viv2DDRI3Authorise(Viv2DPtr v2d, int fd)
{
	int ret;
	struct stat st;
	drm_magic_t magic;

	if (fstat(fd, &st) || !S_ISCHR(st.st_mode))
		return FALSE;

	/*
	 * If the device is a render node, we don't need to auth it.
	 * Render devices start at minor number 128 and up, though it
	 * would be nice to have some other test for this.
	 */
	if (st.st_rdev & 0x80)
		return TRUE;

	ret = drmGetMagic(fd, &magic);
	if (ret){
		VIV2D_ERR_MSG("Viv2DDRI3Open cannot get magic : %d",ret);
		return FALSE;
	}

	ret = drmAuthMagic(v2d->fd, magic);
	if(ret) {
		VIV2D_ERR_MSG("Viv2DDRI3Open cannot auth magic : %d",ret);
		return FALSE;
	}

	return TRUE;
}

static int Viv2DDRI3Open(ScreenPtr pScreen, RRProviderPtr provider, int *o)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	int fd;

	fd = open(v2d->render_node, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		VIV2D_ERR_MSG("Viv2DDRI3Open cannot open %s", v2d->render_node);
		return BadAlloc;
	} else {
		VIV2D_INFO_MSG("Viv2DDRI3Open %s opened in %d", v2d->render_node, fd);
	}

	if (!Viv2DDRI3Authorise(v2d, fd)) {
		VIV2D_ERR_MSG("Viv2DDRI3Open cannot authorize %d", fd);
		close(fd);
		return BadMatch;
	}

	*o = fd;

	return Success;
}

static PixmapPtr Viv2DDRI3PixmapFromFD(ScreenPtr pScreen, int fd,
                                       CARD16 width, CARD16 height, CARD16 stride, CARD8 depth, CARD8 bpp)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	Viv2DPixmapPrivPtr vpix;
	PixmapPtr pixmap;

	pixmap = pScreen->CreatePixmap(pScreen, 0, 0, depth, 0);
	if (pixmap == NullPixmap)
		return pixmap;

	pScreen->ModifyPixmapHeader(pixmap, width, height, depth, bpp, stride, NULL);

	vpix = exaGetPixmapDriverPrivate(pixmap);
	if(vpix) {
		if(vpix->bo)
			etna_bo_del(vpix->bo);

		vpix->bo = etna_bo_from_dmabuf(v2d->dev, fd);
		Viv2DSetFormat(depth, bpp, &vpix->format);

	} else {
		pScreen->DestroyPixmap(pixmap);
		return NullPixmap;
	}

	return pixmap;
}

static int Viv2DDRI3FDFromPixmap(ScreenPtr pScreen, PixmapPtr pixmap,
                                 CARD16 *stride, CARD32 *size)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	Viv2DPixmapPrivPtr vPix = exaGetPixmapDriverPrivate(pixmap);

	/* Only support pixmaps backed by an etnadrm bo */
	if (!vPix || !vPix->bo)
		return BadMatch;

	*stride = pixmap->devKind;
	*size = etna_bo_size(vPix->bo);

	return etna_bo_dmabuf(vPix->bo);
}

static dri3_screen_info_rec etnaviv_dri3_info = {
	.version = 0,
	.open = Viv2DDRI3Open,
	.pixmap_from_fd = Viv2DDRI3PixmapFromFD,
	.fd_from_pixmap = Viv2DDRI3FDFromPixmap,
};

Bool Viv2DDRI3ScreenInit(ScreenPtr pScreen)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	struct stat st;
	char buf[64];

	if (!v2d)
		return FALSE;

	free((void *)v2d->render_node);

	if (fstat(v2d->fd, &st) || !S_ISCHR(st.st_mode))
		return FALSE;

	snprintf(buf, sizeof(buf), "%s/card%d", DRM_DIR_NAME,
	         (unsigned int)st.st_rdev & 0x7f);

	if (access(buf, F_OK))
		return FALSE;

	v2d->render_node = strdup(buf);
	if (!v2d->render_node)
		return FALSE;

	if (!miSyncShmScreenInit(pScreen))
		return FALSE;

	return dri3_screen_init(pScreen, &etnaviv_dri3_info);
}
