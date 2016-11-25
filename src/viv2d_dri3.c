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

static Viv2DPixmapPrivPtr Viv2DAllocPixmap(PixmapPtr pixmap,
	Viv2DFormat fmt)
{
	Viv2DPixmapPrivPtr vpix;

	vpix = calloc(1, sizeof *vpix);
	if (vpix) {
		vpix->width = pixmap->drawable.width;
		vpix->height = pixmap->drawable.height;
		vpix->pitch = pixmap->devKind;
		vpix->format = fmt;
	}
	return vpix;
}

static Viv2DPixmapPrivPtr Viv2DPixmapAttachDmabuf(
	Viv2DPtr v2d, PixmapPtr pixmap, Viv2DFormat fmt,
	int fd)
{
	Viv2DPixmapPrivPtr vpix;
	struct etna_bo *bo;

	bo = etna_bo_from_dmabuf(v2d->dev, fd);
	if (!bo) {
		xf86Msg(X_ERROR,
			   "Viv2D: gpu dmabuf map failed");
			   
			return NULL;
	}

	vpix = Viv2DAllocPixmap(pixmap, fmt);
	if (!vpix) {
		etna_bo_del(bo);
		return NULL;
	}

	vpix->bo = bo;

//	Viv2DSetPixmapPriv(pixmap, vpix);

	return vpix;
}

PixmapPtr Viv2DPixmapFromDmabuf(ScreenPtr pScreen, int fd,
        CARD16 width, CARD16 height, CARD16 stride, CARD8 depth, CARD8 bpp)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	Viv2DFormat fmt;
	PixmapPtr pixmap;

	if (!Viv2DSetFormat(depth, bpp, &fmt))
		return NullPixmap;

	pixmap = pScreen->CreatePixmap(pScreen, 0, 0, depth, 0);
	if (pixmap == NullPixmap)
		return pixmap;

	pScreen->ModifyPixmapHeader(pixmap, width, height, 0, 0, stride, NULL);

	if (!Viv2DPixmapAttachDmabuf(v2d, pixmap, fmt, fd)) {
		pScreen->DestroyPixmap(pixmap);
		return NullPixmap;
	}

	return pixmap;
}

static Bool Viv2DDRI3Authorise(Viv2DPtr v2d, int fd)
{
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

	return drmGetMagic(fd, &magic) == 0 &&
	       drmAuthMagic(v2d->fd, magic) == 0;
}

static int Viv2DDRI3Open(ScreenPtr pScreen, RRProviderPtr provider, int *o)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	int fd;

	fd = open(v2d->render_node, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		return BadAlloc;

	if (!Viv2DDRI3Authorise(v2d, fd)) {
		close(fd);
		return BadMatch;
	}

	*o = fd;

	return Success;
}

static PixmapPtr Viv2DDRI3PixmapFromFD(ScreenPtr pScreen, int fd,
	CARD16 width, CARD16 height, CARD16 stride, CARD8 depth, CARD8 bpp)
{
	return Viv2DPixmapFromDmabuf(pScreen, fd, width, height,
					  stride, depth, bpp);
}

static int Viv2DDRI3FDFromPixmap(ScreenPtr pScreen, PixmapPtr pixmap,
	CARD16 *stride, CARD32 *size)
{
	Viv2DPtr v2d = Viv2DPrivFromScreen(pScreen);
	Viv2DPixmapPrivPtr vPix = Viv2DPrivFromPixmap(pixmap);

	/* Only support pixmaps backed by an etnadrm bo */
	if (!vPix || !vPix->bo)
		return BadMatch;

	*stride = pixmap->devKind;
	*size = etna_bo_size(vPix->bo);

	return etna_bo_to_dmabuf(v2d->dev, vPix->bo);
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
