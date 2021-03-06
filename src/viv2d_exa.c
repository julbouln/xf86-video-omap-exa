
/*
 * Copyright © 2016 Julien Boulnois
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Julien Boulnois <jboulnois@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "omap_driver.h"
#include "omap_exa.h"
#include <omap_drmif.h>

#include "etnaviv_drmif.h"
#include "etnaviv_drm.h"

#include "exa.h"

#include "viv2d.h"
#include "viv2d_exa.h"
#include "viv2d_op.h"


#define VIV2D_STREAM_SIZE 512

#define VIV2D_SOLID 1
#define VIV2D_COPY 1
#define VIV2D_COMPOSITE 1

#define VIV2D_MASK_SUPPORT 1
#define VIV2D_SOLID_PICTURE 1
#define VIV2D_REPEAT 1
#define VIV2D_UPLOAD_TO_SCREEN 1
#define VIV2D_REPEAT_WITH_MASK 1

#define VIV2D_MIN_HW_HEIGHT 64
#define VIV2D_MIN_HW_SIZE_24BIT (128 * 128)

static Viv2DBlendOp viv2d_blend_op[] = {
	{PictOpClear,			DE_BLENDMODE_ZERO, 				DE_BLENDMODE_ZERO},
	{PictOpSrc,				DE_BLENDMODE_ONE, 				DE_BLENDMODE_ZERO},
	{PictOpDst,				DE_BLENDMODE_ZERO, 				DE_BLENDMODE_ONE},
	{PictOpOver,			DE_BLENDMODE_ONE,				DE_BLENDMODE_INVERSED},
	{PictOpOverReverse,		DE_BLENDMODE_INVERSED, 			DE_BLENDMODE_ONE},
	{PictOpIn,				DE_BLENDMODE_NORMAL,			DE_BLENDMODE_ZERO},
	{PictOpInReverse,		DE_BLENDMODE_ZERO,				DE_BLENDMODE_NORMAL},
	{PictOpOut,				DE_BLENDMODE_INVERSED,			DE_BLENDMODE_ZERO},
	{PictOpOutReverse,		DE_BLENDMODE_ZERO,				DE_BLENDMODE_INVERSED},
	{PictOpAtop,			DE_BLENDMODE_NORMAL,			DE_BLENDMODE_INVERSED},
	{PictOpAtopReverse,		DE_BLENDMODE_INVERSED,			DE_BLENDMODE_NORMAL},
	{PictOpXor,				DE_BLENDMODE_INVERSED,			DE_BLENDMODE_INVERSED},
	{PictOpAdd,				DE_BLENDMODE_ONE,				DE_BLENDMODE_ONE},
	{PictOpSaturate,		DE_BLENDMODE_SATURATED_ALPHA,	DE_BLENDMODE_ONE} // ?
};

#define NO_PICT_FORMAT -1
/**
 * Picture Formats and their counter parts
 */
#define VIV2D_PICT_FORMAT_COUNT 8
static const Viv2DFormat
viv2d_pict_format[] = {
	{PICT_a8r8g8b8, 32, DE_FORMAT_A8R8G8B8, DE_SWIZZLE_ARGB, 8},
	{PICT_x8r8g8b8, 32, DE_FORMAT_X8R8G8B8, DE_SWIZZLE_ARGB, 0},
	{PICT_a8b8g8r8, 32, DE_FORMAT_A8R8G8B8, DE_SWIZZLE_ABGR, 8},
	{PICT_x8b8g8r8, 32, DE_FORMAT_X8R8G8B8,	DE_SWIZZLE_ABGR, 0},
	{PICT_b8g8r8a8, 32, DE_FORMAT_A8R8G8B8,	DE_SWIZZLE_BGRA, 8},
	{PICT_b8g8r8x8, 32, DE_FORMAT_X8R8G8B8,	DE_SWIZZLE_BGRA, 8},
	{PICT_r5g6b5, 16, DE_FORMAT_R5G6B5, DE_SWIZZLE_ARGB, 0},
	{PICT_b5g6r5, 16, DE_FORMAT_R5G6B5,	DE_SWIZZLE_ABGR, 0},
	{PICT_a1r5g5b5, 16, DE_FORMAT_A1R5G5B5, DE_SWIZZLE_ARGB, 1},
	{PICT_x1r5g5b5, 16, DE_FORMAT_X1R5G5B5, DE_SWIZZLE_ARGB, 0},
	{PICT_a1b5g5r5, 16, DE_FORMAT_A1R5G5B5,	DE_SWIZZLE_ABGR, 1},
	{PICT_x1b5g5r5, 16,	DE_FORMAT_X1R5G5B5, DE_SWIZZLE_ABGR, 0},
	{PICT_a4r4g4b4, 16, DE_FORMAT_A4R4G4B4, DE_SWIZZLE_ARGB, 4},
	{PICT_x4r4g4b4, 16, DE_FORMAT_X4R4G4B4, DE_SWIZZLE_ARGB, 0},
	{PICT_a4b4g4r4, 16, DE_FORMAT_A4R4G4B4, DE_SWIZZLE_ABGR, 4},
	{PICT_x4b4g4r4, 16, DE_FORMAT_X4R4G4B4, DE_SWIZZLE_ABGR, 0},
	{PICT_a8, 8, DE_FORMAT_A8, 8},
	{PICT_c8, 8, DE_FORMAT_INDEX8, 8},
	{NO_PICT_FORMAT, 0, 0, 0}
	/*END*/
};


static const char *
pix_op_name(int op) {
	switch (op) {
	case PictOpClear:
	case PictOpSrc: return "OpSrc";
	case PictOpDst: return "OpDst";
	case PictOpOver: return "OpOver";
	case PictOpOverReverse: return "OpOverReverse";
	case PictOpIn: return "OpIn";
	case PictOpInReverse: return "OpInReverse";
	case PictOpOut: return "OpOut";
	case PictOpOutReverse: return "OpOutReverse";
	case PictOpAtop: return "OpAtop";
	case PictOpAtopReverse: return "OpAtopReverse";
	case PictOpXor: return "OpXor";
	case PictOpAdd: return "OpAdd";
	case PictOpSaturate: return "OpSaturate";
	}
}
// for debug
static const char *
pix_format_name (pixman_format_code_t format)
{
	switch (format)
	{
	/* 32bpp formats */
	case PIXMAN_a8r8g8b8: return "a8r8g8b8";
	case PIXMAN_x8r8g8b8: return "x8r8g8b8";
	case PIXMAN_a8b8g8r8: return "a8b8g8r8";
	case PIXMAN_x8b8g8r8: return "x8b8g8r8";
	case PIXMAN_b8g8r8a8: return "b8g8r8a8";
	case PIXMAN_b8g8r8x8: return "b8g8r8x8";
	case PIXMAN_r8g8b8a8: return "r8g8b8a8";
	case PIXMAN_r8g8b8x8: return "r8g8b8x8";
	case PIXMAN_x14r6g6b6: return "x14r6g6b6";
	case PIXMAN_x2r10g10b10: return "x2r10g10b10";
	case PIXMAN_a2r10g10b10: return "a2r10g10b10";
	case PIXMAN_x2b10g10r10: return "x2b10g10r10";
	case PIXMAN_a2b10g10r10: return "a2b10g10r10";

	/* sRGB formats */
	case PIXMAN_a8r8g8b8_sRGB: return "a8r8g8b8_sRGB";

	/* 24bpp formats */
	case PIXMAN_r8g8b8: return "r8g8b8";
	case PIXMAN_b8g8r8: return "b8g8r8";

	/* 16bpp formats */
	case PIXMAN_r5g6b5: return "r5g6b5";
	case PIXMAN_b5g6r5: return "b5g6r5";

	case PIXMAN_a1r5g5b5: return "a1r5g5b5";
	case PIXMAN_x1r5g5b5: return "x1r5g5b5";
	case PIXMAN_a1b5g5r5: return "a1b5g5r5";
	case PIXMAN_x1b5g5r5: return "x1b5g5r5";
	case PIXMAN_a4r4g4b4: return "a4r4g4b4";
	case PIXMAN_x4r4g4b4: return "x4r4g4b4";
	case PIXMAN_a4b4g4r4: return "a4b4g4r4";
	case PIXMAN_x4b4g4r4: return "x4b4g4r4";

	/* 8bpp formats */
	case PIXMAN_a8: return "a8";
	case PIXMAN_r3g3b2: return "r3g3b2";
	case PIXMAN_b2g3r3: return "b2g3r3";
	case PIXMAN_a2r2g2b2: return "a2r2g2b2";
	case PIXMAN_a2b2g2r2: return "a2b2g2r2";

#if 0
	case PIXMAN_x4c4: return "x4c4";
	case PIXMAN_g8: return "g8";
#endif
	case PIXMAN_c8: return "x4c4 / c8";
	case PIXMAN_x4g4: return "x4g4 / g8";

	case PIXMAN_x4a4: return "x4a4";

	/* 4bpp formats */
	case PIXMAN_a4: return "a4";
	case PIXMAN_r1g2b1: return "r1g2b1";
	case PIXMAN_b1g2r1: return "b1g2r1";
	case PIXMAN_a1r1g1b1: return "a1r1g1b1";
	case PIXMAN_a1b1g1r1: return "a1b1g1r1";

	case PIXMAN_c4: return "c4";
	case PIXMAN_g4: return "g4";

	/* 1bpp formats */
	case PIXMAN_a1: return "a1";

	case PIXMAN_g1: return "g1";

	/* YUV formats */
	case PIXMAN_yuy2: return "yuy2";
	case PIXMAN_yv12: return "yv12";
	};


	return "<unknown format>";
};
// others utils

static int VIV2DDetectDevice(const char *name)
{
	drmVersionPtr version;
	char buf[64];
	int minor, fd, rc;

	for (minor = 0; minor < 64; minor++) {
		snprintf(buf, sizeof(buf), "%s/card%d", DRM_DIR_NAME,
		         minor);

		fd = open(buf, O_RDWR);
		if (fd == -1)
			continue;

		version = drmGetVersion(fd);
		if (version) {
			rc = strcmp(version->name, name);
			drmFreeVersion(version);

			if (rc == 0) {
				VIV2D_INFO_MSG("VIV2DDetectDevice %s found at %s", name, buf);
				return fd;
			}
		}

		close(fd);
	}

	return -1;
}

static inline unsigned int Viv2DPitch(unsigned width, unsigned bpp)
{
	unsigned pitch = bpp != 4 ? width * ((bpp + 7) / 8) : width / 2;

	/* GC320 and GC600 needs pitch aligned to 16 */
	return ALIGN(pitch, 16);
}

static void *
Viv2DCreatePixmap (ScreenPtr pScreen, int width, int height,
                   int depth, int usage_hint, int bitsPePixrel,
                   int *new_fb_pitch)
{
	Viv2DPixmapPrivPtr priv = calloc(sizeof(Viv2DPixmapPrivRec), 1);
	return priv;
}

static void
Viv2DDestroyPixmap(ScreenPtr pScreen, void *driverPriv)
{
	Viv2DPixmapPrivPtr priv = driverPriv;
	Viv2DRec *v2d = Viv2DPrivFromScreen(pScreen);

//	VIV2D_DBG_MSG("Viv2DDestroyPixmap");

	if (priv->bo && v2d->bo != priv->bo) {
//		VIV2D_DBG_MSG("Viv2DDestroyPixmap freeing bo %p/%p", driverPriv, etna_bo_map(priv->bo));
		etna_bo_del(priv->bo);
	}

	free(priv);
}

static Bool
Viv2DModifyPixmapHeader(PixmapPtr pPixmap, int width, int height,
                        int depth, int bitsPerPixel, int devKind,
                        pointer pPixData)
{
	Viv2DPixmapPrivPtr priv = exaGetPixmapDriverPrivate(pPixmap);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pPixmap);
	ScrnInfoPtr pScrn = pix2scrn(pPixmap);
	OMAPPtr pOMAP = OMAPPTR(pScrn);

	uint32_t size, flags = ETNA_BO_WC;
	Bool ret;

	ret = miModifyPixmapHeader(pPixmap, width, height, depth,
	                           bitsPerPixel, devKind, pPixData);
	if (!ret) {
		return ret;
	}

//	VIV2D_DBG_MSG("Viv2DModifyPixmapHeader %p <=> %p", pPixData, etna_bo_map(v2d->bo));

	priv->width = width	= pPixmap->drawable.width;
	priv->height = height	= pPixmap->drawable.height;
	depth	= pPixmap->drawable.depth;
	bitsPerPixel = pPixmap->drawable.bitsPerPixel;

//	priv->pitch = pPixmap->devKind = Viv2DPitch(width, bitsPerPixel);
	priv->pitch = pPixmap->devKind = OMAPCalculateStride(width, bitsPerPixel);
//	Viv2DSetFormat(pPixmap->drawable.depth, pPixmap->drawable.bitsPerPixel, &priv->format);

	if (pPixData == omap_bo_map(pOMAP->scanout)) {
		VIV2D_DBG_MSG("Viv2DModifyPixmapHeader wrapping scanout buffer pPixData:%p etna:%p omap:%p", pPixData, etna_bo_map(v2d->bo), omap_bo_map(pOMAP->scanout));
		priv->bo = v2d->bo;
		return TRUE;
	} else if (pPixData) {
		/* we can't accelerate this pixmap, and don't ever want to
		 * see it again..
		 */
		pPixmap->devPrivate.ptr = pPixData;
		pPixmap->devKind = devKind;

		/* scratch-pixmap (see GetScratchPixmapHeader()) gets recycled,
		 * so could have a previous bo!
		 */
		if (priv->bo) {
//			VIV2D_DBG_MSG("Viv2DModifyPixmapHeader freeing bo %p/%p", priv, etna_bo_map(priv->bo));
			etna_bo_del(priv->bo);
		}
		priv->bo = NULL;

		return FALSE;
	}
//	VIV2D_DBG_MSG("Viv2DModifyPixmapHeader %dx%d %d %p",
//	         width, height, depth,
//	         pPixData);
//	VIV2D_DBG_MSG("Viv2DModifyPixmapHeader(2) %dx%d %d",
//	         pPixmap->drawable.width, pPixmap->drawable.height, pPixmap->drawable.depth);

	/* passed in values could be zero, indicating that existing values
	 * should be kept.. miModifyPixmapHeader() will deal with that, but
	 * we need to resync to ensure we have the right values in the rest
	 * of this function
	 */
	size = pPixmap->devKind * height;

	if ((!priv->bo) || (etna_bo_size(priv->bo) != size)) {
		/* re-allocate buffer! */
		if (priv->bo)
			etna_bo_del(priv->bo);

		priv->bo = etna_bo_new(v2d->dev, size, flags);
		priv->tiled = FALSE;
	}

	if (!priv->bo) {
		ERROR_MSG("failed to allocate %dx%d bo, size=%d, flags=%08x",
		          width, height, size, flags);
	}

	return priv->bo != NULL;
}


/**
 * UploadToScreen() loads a rectangle of data from src into pDst.
 *
 * @param pDst destination pixmap
 * @param x destination X coordinate.
 * @param y destination Y coordinate
 * @param width width of the rectangle to be copied
 * @param height height of the rectangle to be copied
 * @param src pointer to the beginning of the source data
 * @param src_pitch pitch (in bytes) of the lines of source data.
 *
 * UploadToScreen() copies data in system memory beginning at src (with
 * pitch src_pitch) into the destination pixmap from (x, y) to
 * (x + width, y + height).  This is typically done with hostdata uploads,
 * where the CPU sets up a blit command on the hardware with instructions
 * that the blit data will be fed through some sort of aperture on the card.
 *
 * If UploadToScreen() is performed asynchronously, it is up to the driver
 * to call exaMarkSync().  This is in contrast to most other acceleration
 * calls in EXA.
 *
 * UploadToScreen() can aid in pixmap migration, but is most important for
 * the performance of exaGlyphs() (antialiased font drawing) by allowing
 * pipelining of data uploads, avoiding a sync of the card after each glyph.
 *
 * @return TRUE if the driver successfully uploaded the data.  FALSE
 * indicates that EXA should fall back to doing the upload in software.
 *
 * UploadToScreen() is not required, but is recommended if Composite
 * acceleration is supported.
 */
// WIP
Bool Viv2DUploadToScreen(PixmapPtr pDst,
                         int x,
                         int y, int w, int h, char *src, int src_pitch) {

	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pDst);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDst);
	Viv2DRect rects[2];
	int height = h;
	int bytesPerPixel = (pDst->drawable.bitsPerPixel + 7) / 8;
	Viv2DPixmapPrivPtr srcp;
	ScreenPtr pScreen = pDst->drawable.pScreen;

	if (pDst->drawable.width * pDst->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_UNSUPPORTED_MSG("Viv2DUploadToScreen dest drawable is too small %dx%d", pDst->drawable.width, pDst->drawable.height);
		return FALSE;
	}

	VIV2D_DBG_MSG("Viv2DUploadToScreen check %p(%d) %dx%d(%dx%d) %dx%d %d/%d", src, src_pitch, x, y, w, h,
	              pDst->drawable.width, pDst->drawable.height,
	              pDst->drawable.depth, pDst->drawable.bitsPerPixel);

	Viv2DPixmapPrivRec pix;
	pix.width = w;
	pix.height = h;
	pix.pitch = OMAPCalculateStride(w, pDst->drawable.bitsPerPixel);
//	pix.pitch = Viv2DPitch(w, pDst->drawable.bitsPerPixel);

	pix.priv = NULL;
	pix.bo = NULL;

	srcp = &pix;

	if (!Viv2DSetFormat(pDst->drawable.depth, pDst->drawable.bitsPerPixel, &dst->format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DUploadToScreen unsupported dst format %d/%d %p", pDst->drawable.depth, pDst->drawable.bitsPerPixel, src);
		return FALSE;
	}

	if (!Viv2DSetFormat(pDst->drawable.depth, pDst->drawable.bitsPerPixel, &pix.format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DUploadToScreen unsupported format %d/%d %p", pDst->drawable.depth, pDst->drawable.bitsPerPixel, src);
		return FALSE;
	}


	if (dst->format.fmt == DE_FORMAT_A8)
	{
		VIV2D_UNSUPPORTED_MSG("Viv2DUploadToScreen unsupported dst A8");
		return FALSE;
	}

	int pitch = srcp->pitch;

#ifdef VIV2D_UPLOAD_USERPTR
	int size = ALIGN(pitch * h, 4096);
	void *mem = NULL;
	Bool aligned = FALSE;

//	if (ALIGN((uintptr_t)src, 4096) == (uintptr_t)src)
//		aligned = TRUE;

	VIV2D_DBG_MSG("Viv2DUploadToScreen %p %dx%d(%dx%d) %d/%d/%d %d %d", src, x, y, w, h,
	              src_pitch, srcp->pitch, pitch, dst->pitch, w * bytesPerPixel);

	char *src_buf = src;
	char *buf;

	if (!aligned) {
		posix_memalign(&mem, 4096, size); // aligned buf with aligned size

		buf = mem;
//	char *buf = (char *) etna_bo_map(srcp->bo);

		// just copy the needed rect (w x h)
		while (height--) {
			memcpy(buf, src_buf, w * bytesPerPixel);
//		memcpy(buf, src_buf, pitch);
			src_buf += src_pitch;
			buf += pitch;
		}
	} else {
		mem = src;
	}

	/*	int i;
			for (i = 0, buf = mem; i < height; i++, buf += pitch)
				memcpy(buf, src + src_pitch * i, pitch);
	*/
	struct drm_etnaviv_gem_userptr ureq = {
		.user_ptr = (uintptr_t)mem,
		.user_size = size,
		.flags = ETNA_USERPTR_READ,
	};

	int err = drmCommandWriteRead(v2d->fd, DRM_ETNAVIV_GEM_USERPTR, &ureq, sizeof(ureq));
	if (err) {
		VIV2D_ERR_MSG("Viv2DUploadToScreen userptr fail %d", err);
		return FALSE;
	}
	VIV2D_DBG_MSG("Viv2DUploadToScreen userptr opened %d", ureq.handle);

#ifdef ETNA_BO_FROM_HANDLE_MISSING
	/* !!! we need access to private struct etna_bo since etna_bo_from_handle is missing !!! */
//	srcp->bo = etna_bo_new(v2d->dev, size, ETNA_BO_UNCACHED);

	srcp->bo = etna_bo_alloc(v2d->dev);
	srcp->bo->size = size;
	srcp->bo->handle = ureq.handle;
#else
//	srcp->bo = etna_bo_from_handle(v2d->dev, req.handle, size);
#endif
	VIV2D_DBG_MSG("Viv2DUploadToScreen bo attached %d", ureq.handle);

#else

	int size = pitch * srcp->height;
	srcp->bo = etna_bo_new(v2d->dev, size, ETNA_BO_UNCACHED);

	char *src_buf = src ;
	char *buf = (char *) etna_bo_map(srcp->bo);

	while (height--) {
//		memcpy(buf, src_buf, w * bytesPerPixel);
		memcpy(buf, src_buf, pitch);
		src_buf += src_pitch;
		buf += pitch;
	}

#endif

	rects[0].x1 = x;
	rects[0].y1 = y;
	rects[0].x2 = x + w;
	rects[0].y2 = y + h;

	Viv2DRect clip;
	clip.x1 = x;
	clip.y1 = y;
	clip.x2 = x + w;
	clip.y2 = y + h;

	_Viv2DStreamSrc(v2d, srcp, 0, 0, srcp->width, srcp->height, FALSE); // tmp source
	_Viv2DStreamDst(v2d, dst, VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT, NULL);
	_Viv2DStreamBlendOp(v2d, NULL, 0, 0, FALSE, FALSE);
	_Viv2DStreamRects(v2d, rects, 1);
	_Viv2DStreamCommit(v2d);

//	VIV2D_DBG_MSG("Viv2DUploadToScreen blit done %p", src);

#ifdef VIV2D_UPLOAD_USERPTR
	if (!aligned)
		free(mem);
	free(srcp->bo);
#else
	etna_bo_del(srcp->bo);
#endif
//	VIV2D_DBG_MSG("Viv2DUploadToScreen exit %p", src);
	return TRUE;
}

/**
 * DownloadFromScreen() loads a rectangle of data from pSrc into dst
 *
 * @param pSrc source pixmap
 * @param x source X coordinate.
 * @param y source Y coordinate
 * @param width width of the rectangle to be copied
 * @param height height of the rectangle to be copied
 * @param dst pointer to the beginning of the destination data
 * @param dst_pitch pitch (in bytes) of the lines of destination data.
 *
 * DownloadFromScreen() copies data from offscreen memory in pSrc from
 * (x, y) to (x + width, y + height), to system memory starting at
 * dst (with pitch dst_pitch).  This would usually be done
 * using scatter-gather DMA, supported by a DRM call, or by blitting to AGP
 * and then synchronously reading from AGP.  Because the implementation
 * might be synchronous, EXA leaves it up to the driver to call
 * exaMarkSync() if DownloadFromScreen() was asynchronous.  This is in
 * contrast to most other acceleration calls in EXA.
 *
 * DownloadFromScreen() can aid in the largest bottleneck in pixmap
 * migration, which is the read from framebuffer when evicting pixmaps from
 * framebuffer memory.  Thus, it is highly recommended, even though
 * implementations are typically complicated.
 *
 * @return TRUE if the driver successfully downloaded the data.  FALSE
 * indicates that EXA should fall back to doing the download in software.
 *
 * DownloadFromScreen() is not required, but is highly recommended.
 */
Bool Viv2DDownloadFromScreen(PixmapPtr pSrc,
                             int x, int y,
                             int w, int h, char *dst, int dst_pitch) {
	return FALSE;
}


static inline uint32_t Viv2DScale16(uint32_t val, int bits)
{
	val <<= (16 - bits);
	while (bits < 16) {
		val |= val >> bits;
		bits <<= 1;
	}
	return val >> 8;
}

static inline uint32_t Viv2DColour(Pixel pixel, int depth) {
	uint32_t colour;
	switch (depth) {
	case 15: /* A1R5G5B5 */
		colour = (pixel & 0x8000 ? 0xff000000 : 0) |
		         Viv2DScale16((pixel & 0x7c00) >> 10, 5) << 16 |
		         Viv2DScale16((pixel & 0x03e0) >> 5, 5) << 8 |
		         Viv2DScale16((pixel & 0x001f), 5);
		break;
	case 16: /* R5G6B5 */
		colour = 0xff000000 |
		         Viv2DScale16((pixel & 0xf800) >> 11, 5) << 16 |
		         Viv2DScale16((pixel & 0x07e0) >> 5, 6) << 8 |
		         Viv2DScale16((pixel & 0x001f), 5);
		break;
	case 24: /* A8R8G8B8 */
	default:
		colour = pixel;
		break;
	}
	return colour;
}

#ifdef VIV2D_SOLID
/** @name Solid
 * @{
 */
/**
 * PrepareSolid() sets up the driver for doing a solid fill.
 * @param pPixmap Destination pixmap
 * @param alu raster operation
 * @param planemask write mask for the fill
 * @param fg "foreground" color for the fill
 *
 * This call should set up the driver for doing a series of solid fills
 * through the Solid() call.  The alu raster op is one of the GX*
 * graphics functions listed in X.h, and typically maps to a similar
 * single-byte "ROP" setting in all hardware.  The planemask controls
 * which bits of the destination should be affected, and will only represent
 * the bits up to the depth of pPixmap.  The fg is the pixel value of the
 * foreground color referred to in ROP descriptions.
 *
 * Note that many drivers will need to store some of the data in the driver
 * private record, for sending to the hardware with each drawing command.
 *
 * The PrepareSolid() call is required of all drivers, but it may fail for any
 * reason.  Failure results in a fallback to software rendering.
 */
static Bool Viv2DPrepareSolid (PixmapPtr pPixmap,
                               int alu, Pixel planemask, Pixel fg) {
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pPixmap);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pPixmap);
	uint32_t pitch;

	Viv2DOp *op;

	if (alu != GXcopy) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareSolid unsupported alu %d", alu);
		return FALSE;
	}

	if (!EXA_PM_IS_SOLID(&pPixmap->drawable, planemask)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareSolid unsupported planemask %x", (uint32_t)planemask);
		return FALSE;
	}

	if (pPixmap->drawable.height < VIV2D_MIN_HW_HEIGHT || pPixmap->drawable.width * pPixmap->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareSolid dest drawable is too small %dx%d", pPixmap->drawable.width, pPixmap->drawable.height);
		return FALSE;
	}

	if (!Viv2DSetFormat(pPixmap->drawable.depth, pPixmap->drawable.bitsPerPixel, &dst->format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareSolid unsupported format for depth:%d bpp:%d", pPixmap->drawable.depth, pPixmap->drawable.bitsPerPixel);
		return FALSE;
	}

	pitch = pPixmap->devKind;

	op = _Viv2DOpCreate();
	op->fg = Viv2DColour(fg, pPixmap->drawable.depth);
	op->mask = (uint32_t)planemask;
	op->dst = dst;
	v2d->op = op;

	VIV2D_DBG_MSG("Viv2DPrepareSolid %p %dx%d, %x %x %d, %d %d %p", dst,
	              pPixmap->drawable.width, pPixmap->drawable.height, op->fg ,
	              op->mask, pPixmap->drawable.depth, alu, pitch, dst);

	return TRUE;
}

/**
 * Solid() performs a solid fill set up in the last PrepareSolid() call.
 *
 * @param pPixmap destination pixmap
 * @param x1 left coordinate
 * @param y1 top coordinate
 * @param x2 right coordinate
 * @param y2 bottom coordinate
 *
 * Performs the fill set up by the last PrepareSolid() call, covering the
 * area from (x1,y1) to (x2,y2) in pPixmap.  Note that the coordinates are
 * in the coordinate space of the destination pixmap, so the driver will
 * need to set up the hardware's offset and pitch for the destination
 * coordinates according to the pixmap's offset and pitch within
 * framebuffer.  This likely means using exaGetPixmapOffset() and
 * exaGetPixmapPitch().
 *
 * This call is required if PrepareSolid() ever succeeds.
 */
static void Viv2DSolid (PixmapPtr pPixmap, int x1, int y1, int x2, int y2) {
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pPixmap);
	Viv2DOp *op = v2d->op;
	if (op->cur_rect < VIV2D_MAX_RECTS)
	{
//	VIV2D_DBG_MSG("Viv2DSolid %dx%d:%dx%d %d", x1, y1, x2, y2, solidOp->cur_rect);
		_Viv2DOpAddRect(op, x1, y1, x2 - x1, y2 - y1);
	} else {
		_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_CLEAR, NULL);
		_Viv2DStreamColor(v2d, op->fg);
		_Viv2DStreamRects(v2d, op->rects, op->cur_rect);
		_Viv2DStreamFlush(v2d);
		op->cur_rect = 0;
		_Viv2DOpAddRect(op, x1, y1, x2 - x1, y2 - y1);

//		VIV2D_ERR_MSG("Viv2DSolid rects overflow %d >= %d", op->cur_rect, VIV2D_MAX_RECTS);
	}
}

/**
 * DoneSolid() finishes a set of solid fills.
 *
 * @param pPixmap destination pixmap.
 *
 * The DoneSolid() call is called at the end of a series of consecutive
 * Solid() calls following a successful PrepareSolid().  This allows drivers
 * to finish up emitting drawing commands that were buffered, or clean up
 * state from PrepareSolid().
 *
 * This call is required if PrepareSolid() ever succeeds.
 */
static void Viv2DDoneSolid (PixmapPtr pPixmap) {
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pPixmap);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pPixmap);
	Viv2DOp *op = v2d->op;

	_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_CLEAR, NULL);
	_Viv2DStreamColor(v2d, op->fg);
	_Viv2DStreamRects(v2d, op->rects, op->cur_rect);

	VIV2D_DBG_MSG("Viv2DDoneSolid %d", v2d->stream->offset);

	_Viv2DStreamCommit(v2d);

	_Viv2DOpDestroy(op);
	v2d->op = NULL;
}
/** @} */
#else
static Bool
PrepareSolidFail(PixmapPtr pPixmap, int alu, Pixel planemask, Pixel fill_colour)
{
	return FALSE;
}
#endif

#ifdef VIV2D_COPY
/** @name Copy
 * @{
 */
/**
 * PrepareCopy() sets up the driver for doing a copy within video
 * memory.
 *
 * @param pSrcPixmap source pixmap
 * @param pDstPixmap destination pixmap
 * @param dx X copy direction
 * @param dy Y copy direction
 * @param alu raster operation
 * @param planemask write mask for the fill
 *
 * This call should set up the driver for doing a series of copies from the
 * the pSrcPixmap to the pDstPixmap.  The dx flag will be positive if the
 * hardware should do the copy from the left to the right, and dy will be
 * positive if the copy should be done from the top to the bottom.  This
 * is to deal with self-overlapping copies when pSrcPixmap == pDstPixmap.
 * If your hardware can only support blits that are (left to right, top to
 * bottom) or (right to left, bottom to top), then you should set
 * #EXA_TWO_BITBLT_DIRECTIONS, and EXA will break down Copy operations to
 * ones that meet those requirements.  The alu raster op is one of the GX*
 * graphics functions listed in X.h, and typically maps to a similar
 * single-byte "ROP" setting in all hardware.  The planemask controls which
 * bits of the destination should be affected, and will only represent the
 * bits up to the depth of pPixmap.
 *
 * Note that many drivers will need to store some of the data in the driver
 * private record, for sending to the hardware with each drawing command.
 *
 * The PrepareCopy() call is required of all drivers, but it may fail for any
 * reason.  Failure results in a fallback to software rendering.
 */
static Bool Viv2DPrepareCopy (PixmapPtr pSrcPixmap,
                              PixmapPtr pDstPixmap,
                              int dx, int dy, int alu, Pixel planemask) {
	Viv2DPixmapPrivPtr src = exaGetPixmapDriverPrivate(pSrcPixmap);
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pDstPixmap);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDstPixmap);

	Viv2DOp *op;

	if (alu != GXcopy) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareCopy unsupported alu %d", alu);
		return FALSE;
	}

	if (pDstPixmap->drawable.height < VIV2D_MIN_HW_HEIGHT || pDstPixmap->drawable.width * pDstPixmap->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareCopy dest drawable is too small %dx%d", pDstPixmap->drawable.width, pDstPixmap->drawable.height);
		return FALSE;
	}

	if (!Viv2DSetFormat(pSrcPixmap->drawable.depth, pSrcPixmap->drawable.bitsPerPixel, &src->format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareCopy unsupported format for depth:%d bpp:%d", pSrcPixmap->drawable.depth, pSrcPixmap->drawable.bitsPerPixel);
		return FALSE;
	}
	if (!Viv2DSetFormat(pDstPixmap->drawable.depth, pDstPixmap->drawable.bitsPerPixel, &dst->format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareCopy unsupported format for depth:%d bpp:%d", pDstPixmap->drawable.depth, pDstPixmap->drawable.bitsPerPixel);
		return FALSE;
	}

	if (dst->format.fmt == DE_FORMAT_A8)
	{
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareCopy unsupported dst A8");
		return FALSE;
	}

	op = _Viv2DOpCreate();
	op->mask = (uint32_t)planemask;
	op->src = src;
	op->dst = dst;
	v2d->op = op;

	VIV2D_DBG_MSG("Viv2DPrepareCopy %p(%dx%d) -> %p(%dx%d) %dx%d %d %x",
	              src, src->width, src->height,
	              dst, dst->width, dst->height,
	              dx, dy, alu, (uint32_t)planemask);

	return TRUE;
};

/**
 * Copy() performs a copy set up in the last PrepareCopy call.
 *
 * @param pDstPixmap destination pixmap
 * @param srcX source X coordinate
 * @param srcY source Y coordinate
 * @param dstX destination X coordinate
 * @param dstY destination Y coordinate
 * @param width width of the rectangle to be copied
 * @param height height of the rectangle to be copied.
 *
 * Performs the copy set up by the last PrepareCopy() call, copying the
 * rectangle from (srcX, srcY) to (srcX + width, srcY + width) in the source
 * pixmap to the same-sized rectangle at (dstX, dstY) in the destination
 * pixmap.  Those rectangles may overlap in memory, if
 * pSrcPixmap == pDstPixmap.  Note that this call does not receive the
 * pSrcPixmap as an argument -- if it's needed in this function, it should
 * be stored in the driver private during PrepareCopy().  As with Solid(),
 * the coordinates are in the coordinate space of each pixmap, so the driver
 * will need to set up source and destination pitches and offsets from those
 * pixmaps, probably using exaGetPixmapOffset() and exaGetPixmapPitch().
 *
 * This call is required if PrepareCopy ever succeeds.
 */
static void Viv2DCopy (PixmapPtr pDstPixmap,
                       int srcX,
                       int srcY, int dstX, int dstY, int width, int height) {
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDstPixmap);
	Viv2DOp *op = v2d->op;

	VIV2D_DBG_MSG("Viv2DCopy %p(%dx%d) -> %p(%dx%d) : %dx%d", op->src, srcX, srcY, op->dst, dstX, dstY, width, height);

// new srcX,srcY group
	if (op->prev_src_x != srcX || op->prev_src_y != srcY || op->cur_rect >= VIV2D_MAX_RECTS) {
		// flush previous rects
		if (op->prev_src_x > -1) {
			_Viv2DStreamRects(v2d, op->rects, op->cur_rect);
			_Viv2DStreamFlush(v2d);
			op->cur_rect = 0;
		}

		// create states for srcX,srcY group
		_Viv2DStreamSrc(v2d, op->src, srcX, srcY, width, height, FALSE);
		_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT, NULL);
		_Viv2DStreamBlendOp(v2d, op->blend_op, 0, 0, FALSE, FALSE);
	}

	_Viv2DOpAddRect(op, dstX, dstY, width, height);

	op->prev_src_x = srcX;
	op->prev_src_y = srcY;
}

/**
 * DoneCopy() finishes a set of copies.
 *
 * @param pPixmap destination pixmap.
 *
 * The DoneCopy() call is called at the end of a series of consecutive
 * Copy() calls following a successful PrepareCopy().  This allows drivers
 * to finish up emitting drawing commands that were buffered, or clean up
 * state from PrepareCopy().
 *
 * This call is required if PrepareCopy() ever succeeds.
 */
static void Viv2DDoneCopy (PixmapPtr pDstPixmap) {
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pDstPixmap);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDstPixmap);
	Viv2DOp *op = v2d->op;

	_Viv2DStreamRects(v2d, op->rects, op->cur_rect);

	VIV2D_DBG_MSG("Viv2DDoneCopy %d", v2d->stream->offset);

	_Viv2DStreamCommit(v2d);

	_Viv2DOpDestroy(op);
	v2d->op = NULL;
}
/** @} */
#else
static Bool
PrepareCopyFail(PixmapPtr pSrc, PixmapPtr pDst, int xdir, int ydir,
                int alu, Pixel planemask)
{
	return FALSE;
}
#endif

#ifdef VIV2D_COMPOSITE

static PixmapPtr
GetDrawablePixmap(DrawablePtr pDrawable) {
	/* Make sure there is a drawable. */
	if (NULL == pDrawable) {
		return NULL;
	}

	/* Check for a backing pixmap. */
	if (DRAWABLE_WINDOW == pDrawable->type) {

		WindowPtr pWindow = (WindowPtr) pDrawable;
		return pDrawable->pScreen->GetWindowPixmap(pWindow);
	}

	/* Otherwise, it's a regular pixmap. */
	return (PixmapPtr) pDrawable;
}

static Bool Viv2DGetPictureFormat(int exa_fmt, Viv2DFormat *fmt) {
	int i;
	Bool isFound = FALSE;
	int size = VIV2D_PICT_FORMAT_COUNT;

	for (i = 0; i < size && !isFound; i++) {
		if (exa_fmt == viv2d_pict_format[i].exaFmt) {
			*fmt = (viv2d_pict_format[i]);
			isFound = TRUE;
		}
	}
	/*May be somehow usable*/
	if (!isFound) {

		*fmt = viv2d_pict_format[size - 1];
		fmt->exaFmt = exa_fmt;
	}
	return isFound;
}

/**
 * CheckComposite() checks to see if a composite operation could be
 * accelerated.
 *
 * @param op Render operation
 * @param pSrcPicture source Picture
 * @param pMaskPicture mask picture
 * @param pDstPicture destination Picture
 *
 * The CheckComposite() call checks if the driver could handle acceleration
 * of op with the given source, mask, and destination pictures.  This allows
 * drivers to check source and destination formats, supported operations,
 * transformations, and component alpha state, and send operations it can't
 * support to software rendering early on.  This avoids costly pixmap
 * migration to the wrong places when the driver can't accelerate
 * operations.  Note that because migration hasn't happened, the driver
 * can't know during CheckComposite() what the offsets and pitches of the
 * pixmaps are going to be.
 *
 * See PrepareComposite() for more details on likely issues that drivers
 * will have in accelerating Composite operations.
 *
 * The CheckComposite() call is recommended if PrepareComposite() is
 * implemented, but is not required.
 */
static Bool
Viv2DCheckComposite (int op,
                     PicturePtr pSrcPicture,
                     PicturePtr pMaskPicture, PicturePtr pDstPicture) {
	PixmapPtr pSrc = GetDrawablePixmap(pSrcPicture->pDrawable);
	PixmapPtr pDst = GetDrawablePixmap(pDstPicture->pDrawable);

	Viv2DFormat src_fmt;
	Viv2DFormat msk_fmt;
	Viv2DFormat dst_fmt;

	if (pSrc == NULL) {
#ifdef VIV2D_SOLID_PICTURE
		SourcePict *sp = pSrcPicture->pSourcePict;

		if (sp->type == SourcePictTypeSolidFill) {
		} else {
			VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported src is not a drawable : %d", sp->type);
			return FALSE;
		}
#else
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported src is not a drawable");
		return FALSE;
#endif
	}

	if (pDst == NULL) {
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported dest is not a drawable");
		return FALSE;
	}

	/*
		VIV2D_DBG_MSG("Viv2DCheckComposite %d : %dx%d(%s) -> %dx%d(%s)",
		              op, pSrc->drawable.width, pSrc->drawable.height, pix_format_name(pSrcPicture->format),
		              pDst->drawable.width, pDst->drawable.height, pix_format_name(pDstPicture->format)
		             );
	*/
	if (pDst->drawable.height < VIV2D_MIN_HW_HEIGHT || pDst->drawable.width * pDst->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite dest drawable is too small %dx%d", pDst->drawable.width, pDst->drawable.height);
		return FALSE;
	}

	/*For forward compatibility*/
	if (op > PictOpSaturate) {
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite op unsupported : %d", op);
		return FALSE;
	}

	/*Format Checks*/
	if (!Viv2DGetPictureFormat(pSrcPicture->format, &src_fmt)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported src format %s", pix_format_name(pSrcPicture->format));
		return FALSE;
	}

	/*
		if (pMaskPicture != NULL) {
			if (!Viv2DGetPictureFormat(pMaskPicture->format, &msk_fmt)) {
				VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported msk format %s", pix_format_name(pMaskPicture->format));
				return FALSE;
			}
		}
	*/
	if (!Viv2DGetPictureFormat(pDstPicture->format, &dst_fmt)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported dst format %s", pix_format_name(pDstPicture->format));
		return FALSE;
	}

	if (dst_fmt.fmt == DE_FORMAT_A8)
	{
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported dst A8");
		return FALSE;
	}

	if (pMaskPicture && pMaskPicture->transform) {
		return FALSE;
	}

	if (pMaskPicture && pMaskPicture->componentAlpha) {
		return FALSE;
	}

	if (pMaskPicture && pMaskPicture->repeat) {
		return FALSE;
	}

	if ( pSrcPicture->transform ) {
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite transform unsupported");
		return FALSE;
	}

	if ( pSrcPicture->repeat && pSrc) {
#ifdef VIV2D_REPEAT
#ifndef VIV2D_REPEAT_WITH_MASK
		if (pMaskPicture != NULL) {
			VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite repeat with mask unsupported");
			return FALSE;
		}
#endif

		if (pSrc->drawable.width == 1 && pSrc->drawable.height == 1) {
			// 1x1 stretch
		} else {
			VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite repeat > 1x1 unsupported");
			return FALSE;
		}
#else
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite repeat unsupported");
		return FALSE;
#endif
	}

	if ((pMaskPicture != NULL)) {
#ifdef VIV2D_MASK_SUPPORT
		PixmapPtr pMsk = GetDrawablePixmap(pSrcPicture->pDrawable);

		if (pMsk == NULL) {
			VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite unsupported mask is not a drawable");
			return FALSE;
		}

		if ( pMaskPicture->repeat) {
			VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite mask repeat unsupported with mask");
			return FALSE;
		}
#else
		VIV2D_UNSUPPORTED_MSG("Viv2DCheckComposite mask unsupported");
		return FALSE;
#endif
	}

	return TRUE;
}

/**
 * PrepareComposite() sets up the driver for doing a Composite operation
 * described in the Render extension protocol spec.
 *
 * @param op Render operation
 * @param pSrcPicture source Picture
 * @param pMaskPicture mask picture
 * @param pDstPicture destination Picture
 * @param pSrc source pixmap
 * @param pMask mask pixmap
 * @param pDst destination pixmap
 *
 * This call should set up the driver for doing a series of Composite
 * operations, as described in the Render protocol spec, with the given
 * pSrcPicture, pMaskPicture, and pDstPicture.  The pSrc, pMask, and
 * pDst are the pixmaps containing the pixel data, and should be used for
 * setting the offset and pitch used for the coordinate spaces for each of
 * the Pictures.
 *
 * Notes on interpreting Picture structures:
 * - The Picture structures will always have a valid pDrawable.
 * - The Picture structures will never have alphaMap set.
 * - The mask Picture (and therefore pMask) may be NULL, in which case the
 *   operation is simply src OP dst instead of src IN mask OP dst, and
 *   mask coordinates should be ignored.
 * - pMarkPicture may have componentAlpha set, which greatly changes
 *   the behavior of the Composite operation.  componentAlpha has no effect
 *   when set on pSrcPicture or pDstPicture.
 * - The source and mask Pictures may have a transformation set
 *   (Picture->transform != NULL), which means that the source coordinates
 *   should be transformed by that transformation, resulting in scaling,
 *   rotation, etc.  The PictureTransformPoint() call can transform
 *   coordinates for you.  Transforms have no effect on Pictures when used
 *   as a destination.
 * - The source and mask pictures may have a filter set.  PictFilterNearest
 *   and PictFilterBilinear are defined in the Render protocol, but others
 *   may be encountered, and must be handled correctly (usually by
 *   PrepareComposite failing, and falling back to software).  Filters have
 *   no effect on Pictures when used as a destination.
 * - The source and mask Pictures may have repeating set, which must be
 *   respected.  Many chipsets will be unable to support repeating on
 *   pixmaps that have a width or height that is not a power of two.
 *
 * If your hardware can't support source pictures (textures) with
 * non-power-of-two pitches, you should set #EXA_OFFSCREEN_ALIGN_POT.
 *
 * Note that many drivers will need to store some of the data in the driver
 * private record, for sending to the hardware with each drawing command.
 *
 * The PrepareComposite() call is not required.  However, it is highly
 * recommended for performance of antialiased font rendering and performance
 * of cairo applications.  Failure results in a fallback to software
 * rendering.
 */
static Bool
Viv2DPrepareComposite(int rop, PicturePtr pSrcPicture,
                      PicturePtr pMaskPicture,
                      PicturePtr pDstPicture,
                      PixmapPtr pSrc, PixmapPtr pMask, PixmapPtr pDst) {
	Viv2DPixmapPrivPtr src = NULL;
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pDst);
	Viv2DPixmapPrivPtr msk = NULL;
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDst);
	Viv2DOp *op;

	if (pSrc != NULL)
		src = exaGetPixmapDriverPrivate(pSrc);

	if (!Viv2DGetPictureFormat(pSrcPicture->format, &src->format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareComposite unsupported src format %s", pix_format_name(pSrcPicture->format));
		return FALSE;
	}

	if (!Viv2DGetPictureFormat(pDstPicture->format, &dst->format)) {
		VIV2D_UNSUPPORTED_MSG("Viv2DPrepareComposite unsupported dst format %s", pix_format_name(pDstPicture->format));
		return FALSE;
	}

	if (pMaskPicture != NULL) {
		msk = exaGetPixmapDriverPrivate(pMask);

		if (!Viv2DGetPictureFormat(pMaskPicture->format, &msk->format)) {
			VIV2D_UNSUPPORTED_MSG("Viv2DPrepareComposite unsupported msk format %s", pix_format_name(pMaskPicture->format));
			return FALSE;
		}

		VIV2D_DBG_MSG("Viv2DPrepareComposite msk:%p(%dx%d) %s",
		              msk, msk->width, msk->height, pix_format_name(pMaskPicture->format));
	}
	VIV2D_DBG_MSG("Viv2DPrepareComposite src:%p(%dx%d) %s -> dst:%p(%dx%d) %s / op:%d(%s)",
	              src, src->width, src->height, pix_format_name(pSrcPicture->format),
	              dst, dst->width, dst->height, pix_format_name(pDstPicture->format), rop, pix_op_name(rop));

	op = _Viv2DOpCreate();
	op->blend_op = &viv2d_blend_op[rop];

	op->src_alpha_mode_global = FALSE;
	op->dst_alpha_mode_global = FALSE;
	op->src_alpha = 0x00;
	op->dst_alpha = 0x00;

	if (Viv2DFixNonAlpha(&src->format)) {
		op->src_alpha_mode_global = TRUE;
		op->src_alpha = 0xff;
	}

	if (Viv2DFixNonAlpha(&dst->format)) {
		op->dst_alpha_mode_global = TRUE;
		op->dst_alpha = 0xff;
	}

	// FIXME for an unknown reason, alpha src to fixed dest does not work propertly with 24bits depth
/*	if (op->dst_alpha_mode_global && !op->src_alpha_mode_global) {
		return FALSE;
	}
*/
	op->src = src;
	op->dst = dst;
	op->msk = msk;

	op->src_type = viv2d_src_pix;

	if (pSrcPicture->repeat && pSrc->drawable.width == 1 && pSrc->drawable.height == 1) {
		op->src_type = viv2d_src_1x1_repeat;
	}

	if (pSrc == NULL && pSrcPicture->pSourcePict->type == SourcePictTypeSolidFill) {
		op->src_type = viv2d_src_solid;
		op->fg = pSrcPicture->pSourcePict->solidFill.color;
	}
	v2d->op = op;

	return TRUE;
}

/**
     * Composite() performs a Composite operation set up in the last
     * PrepareComposite() call.
     *
     * @param pDstPixmap destination pixmap
     * @param srcX source X coordinate
     * @param srcY source Y coordinate
     * @param maskX source X coordinate
     * @param maskY source Y coordinate
     * @param dstX destination X coordinate
     * @param dstY destination Y coordinate
     * @param width destination rectangle width
     * @param height destination rectangle height
     *
     * Performs the Composite operation set up by the last PrepareComposite()
     * call, to the rectangle from (dstX, dstY) to (dstX + width, dstY + height)
     * in the destination Pixmap.  Note that if a transformation was set on
     * the source or mask Pictures, the source rectangles may not be the same
     * size as the destination rectangles and filtering.  Getting the coordinate
     * transformation right at the subpixel level can be tricky, and rendercheck
     * can test this for you.
     *
     * This call is required if PrepareComposite() ever succeeds.
     */
// dest = (source IN mask) OP dest
static void
Viv2DComposite(PixmapPtr pDst, int srcX, int srcY, int maskX, int maskY,
               int dstX, int dstY, int width, int height) {
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDst);
	Viv2DOp *op = v2d->op;

	VIV2D_DBG_MSG("Viv2DComposite src:%dx%d(%dx%d) -> dst:%dx%d(%dx%d) : %dx%d %d",
	              srcX, srcY, op->src->width, op->src->height,
	              dstX, dstY, op->dst->width, op->dst->height,
	              width, height, op->src_type);

	if (op->msk) {
		// tmp 32bits argb pix
		Viv2DPixmapPrivRec tmp;
		Viv2DBlendOp *cpy_op = &viv2d_blend_op[PictOpSrc];
		Viv2DBlendOp *msk_op = &viv2d_blend_op[PictOpInReverse];

		tmp.width = width;
		tmp.height = height;
		tmp.pitch = OMAPCalculateStride(width, 32);
		Viv2DSetFormat(32, 32, &tmp.format);
		tmp.bo = etna_bo_new(v2d->dev, tmp.pitch * tmp.height, ETNA_BO_UNCACHED);

		Viv2DRect mrect;
		mrect.x1 = 0;
		mrect.y1 = 0;
		mrect.x2 = width;
		mrect.y2 = height;

		Viv2DRect drect;
		drect.x1 = dstX;
		drect.y1 = dstY;
		drect.x2 = dstX + width;
		drect.y2 = dstY + height;

		switch (op->src_type) {
		case viv2d_src_1x1_repeat:
			_Viv2DStreamSrc(v2d, op->src, 0, 0, 1, 1, FALSE);
			etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_LOW,
			               VIVS_DE_STRETCH_FACTOR_LOW_X(((op->src->width - 1) << 16) / (op->dst->width - 1)));
			etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_HIGH,
			               VIVS_DE_STRETCH_FACTOR_HIGH_Y(((op->src->height - 1) << 16) / (op->dst->height - 1)));
			_Viv2DStreamDst(v2d, &tmp, VIVS_DE_DEST_CONFIG_COMMAND_STRETCH_BLT, NULL);
			break;
		case viv2d_src_solid:
			_Viv2DStreamDst(v2d, &tmp, VIVS_DE_DEST_CONFIG_COMMAND_CLEAR, NULL);
			_Viv2DStreamColor(v2d, op->fg);
			break;
		default:
			_Viv2DStreamSrc(v2d, op->src, srcX, srcY, width, height, FALSE);
			_Viv2DStreamDst(v2d, &tmp, VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT, NULL);
			break;
		}

		_Viv2DStreamBlendOp(v2d, cpy_op, op->src_alpha, 0, op->src_alpha_mode_global, FALSE);
		_Viv2DStreamRects(v2d, &mrect, 1);

		// apply msk with mask op to tmp
		_Viv2DStreamSrc(v2d, op->msk, maskX, maskY, width, height, FALSE);
		_Viv2DStreamDst(v2d, &tmp, VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT, NULL);
		_Viv2DStreamBlendOp(v2d, msk_op, 0, 0, FALSE, FALSE);
		_Viv2DStreamRects(v2d, &mrect, 1);

		// finally apply to dest
		_Viv2DStreamSrc(v2d, &tmp, 0, 0, width, height, FALSE);
		_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT, NULL);
		_Viv2DStreamBlendOp(v2d, op->blend_op, 0, op->dst_alpha, FALSE, op->dst_alpha_mode_global);
		VIV2D_DBG_MSG("Viv2DComposite mask %x %d %d -> %d\n", op->fg, op->blend_op->op, op->blend_op->srcBlendMode, op->blend_op->dstBlendMode);
		_Viv2DStreamRects(v2d, &drect, 1);

		_Viv2DStreamCommit(v2d); // commit now because of tmp bo
		etna_bo_del(tmp.bo);
	} else {
		// new srcX,srcY group
		if (op->prev_src_x != srcX || op->prev_src_y != srcY || op->cur_rect >= VIV2D_MAX_RECTS) {
			// flush previous rects
			if (op->prev_src_x > -1) {
				_Viv2DStreamRects(v2d, op->rects, op->cur_rect);
				_Viv2DStreamFlush(v2d);
				op->cur_rect = 0;
			}

			// create states for srcX,srcY group
			switch (op->src_type) {
			case viv2d_src_1x1_repeat:
				_Viv2DStreamSrc(v2d, op->src, 0, 0, 1, 1, FALSE);
				etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_LOW,
				               VIVS_DE_STRETCH_FACTOR_LOW_X(((op->src->width - 1) << 16) / (op->dst->width - 1)));
				etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_HIGH,
				               VIVS_DE_STRETCH_FACTOR_HIGH_Y(((op->src->height - 1) << 16) / (op->dst->height - 1)));
				_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_STRETCH_BLT, NULL);
				break;
			case viv2d_src_solid:
				_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_CLEAR, NULL);
				_Viv2DStreamColor(v2d, op->fg);
				break;
			default:
				_Viv2DStreamSrc(v2d, op->src, srcX, srcY, width, height, FALSE);
				_Viv2DStreamDst(v2d, op->dst, VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT, NULL);
				break;
			}
			_Viv2DStreamBlendOp(v2d, op->blend_op, op->src_alpha, op->dst_alpha, op->src_alpha_mode_global, op->dst_alpha_mode_global);
		}

		_Viv2DOpAddRect(op, dstX, dstY, width, height);

		op->prev_src_x = srcX;
		op->prev_src_y = srcY;
	}
}

/**
     * DoneComposite() finishes a set of Composite operations.
     *
     * @param pPixmap destination pixmap.
     *
     * The DoneComposite() call is called at the end of a series of consecutive
     * Composite() calls following a successful PrepareComposite().  This allows
     * drivers to finish up emitting drawing commands that were buffered, or
     * clean up state from PrepareComposite().
     *
     * This call is required if PrepareComposite() ever succeeds.
     */
static void Viv2DDoneComposite (PixmapPtr pDst) {
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDst);
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pDst);
	Viv2DOp *op = v2d->op;

	if (op->msk) {
		// already done masked operations
	} else {
		_Viv2DStreamRects(v2d, op->rects, op->cur_rect);

		VIV2D_DBG_MSG("Viv2DDoneComposite %d", v2d->stream->offset);
		_Viv2DStreamCommit(v2d);
	}
	_Viv2DOpDestroy(op);
	v2d->op = NULL;
}

#else

static Bool
CheckCompositeFail(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
                   PicturePtr pDstPicture)
{
	return FALSE;
}

static Bool
PrepareCompositeFail(int op, PicturePtr pSrcPicture, PicturePtr pMaskPicture,
                     PicturePtr pDstPicture, PixmapPtr pSrc, PixmapPtr pMask, PixmapPtr pDst)
{
	return FALSE;
}

#endif

/**
 * WaitMarker is a required EXA callback but synchronization is
 * performed during OMAPPrepareAccess so this function does not
 * have anything to do at present
 */
static void
Viv2DWaitMarker(ScreenPtr pScreen, int marker)
{
	/* no-op */
}

static inline uint32_t idx2op(int index)
{
	switch (index) {
	case EXA_PREPARE_SRC:
	case EXA_PREPARE_MASK:
	case EXA_PREPARE_AUX_SRC:
	case EXA_PREPARE_AUX_MASK:
		return DRM_ETNA_PREP_READ;
	case EXA_PREPARE_AUX_DEST:
	case EXA_PREPARE_DEST:
	default:
		return DRM_ETNA_PREP_READ | DRM_ETNA_PREP_WRITE;
	}
}

/**
 * PrepareAccess() is called before CPU access to an offscreen pixmap.
 *
 * @param pPix the pixmap being accessed
 * @param index the index of the pixmap being accessed.
 *
 * PrepareAccess() will be called before CPU access to an offscreen pixmap.
 * This can be used to set up hardware surfaces for byteswapping or
 * untiling, or to adjust the pixmap's devPrivate.ptr for the purpose of
 * making CPU access use a different aperture.
 *
 * The index is one of #EXA_PREPARE_DEST, #EXA_PREPARE_SRC,
 * #EXA_PREPARE_MASK, #EXA_PREPARE_AUX_DEST, #EXA_PREPARE_AUX_SRC, or
 * #EXA_PREPARE_AUX_MASK. Since only up to #EXA_NUM_PREPARE_INDICES pixmaps
 * will have PrepareAccess() called on them per operation, drivers can have
 * a small, statically-allocated space to maintain state for PrepareAccess()
 * and FinishAccess() in.  Note that PrepareAccess() is only called once per
 * pixmap and operation, regardless of whether the pixmap is used as a
 * destination and/or source, and the index may not reflect the usage.
 *
 * PrepareAccess() may fail.  An example might be the case of hardware that
 * can set up 1 or 2 surfaces for CPU access, but not 3.  If PrepareAccess()
 * fails, EXA will migrate the pixmap to system memory.
 * DownloadFromScreen() must be implemented and must not fail if a driver
 * wishes to fail in PrepareAccess().  PrepareAccess() must not fail when
 * pPix is the visible screen, because the visible screen can not be
 * migrated.
 *
 * @return TRUE if PrepareAccess() successfully prepared the pixmap for CPU
 * drawing.
 * @return FALSE if PrepareAccess() is unsuccessful and EXA should use
 * DownloadFromScreen() to migate the pixmap out.
 */
static Bool
Viv2DPrepareAccess(PixmapPtr pPixmap, int index)
{
	Viv2DPixmapPrivPtr priv = exaGetPixmapDriverPrivate(pPixmap);
	ScrnInfoPtr pScrn = pix2scrn(pPixmap);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pPixmap);
	int state;
	int prep = idx2op(index);

	pPixmap->devPrivate.ptr = etna_bo_map(priv->bo);
	if (!pPixmap->devPrivate.ptr) {
		return FALSE;
	}

	/* wait for blits complete.. note we could be a bit more clever here
	 * for non-DRI2 buffers and use separate OMAP{Prepare,Finish}GPUAccess()
	 * fxns wrapping accelerated GPU operations.. this way we don't have
	 * to prep/fini around each CPU operation, but only when there is an
	 * intervening GPU operation (or if we go to a stronger op mask, ie.
	 * first CPU access is READ and second is WRITE).
	 */

	state = etna_bo_cpu_prep(priv->bo, prep);

	if (state) {
		return FALSE;
	}

	return TRUE;
}

/**
 * FinishAccess() is called after CPU access to an offscreen pixmap.
 *
 * @param pPix the pixmap being accessed
 * @param index the index of the pixmap being accessed.
 *
 * FinishAccess() will be called after finishing CPU access of an offscreen
 * pixmap set up by PrepareAccess().  Note that the FinishAccess() will not be
 * called if PrepareAccess() failed and the pixmap was migrated out.
 */
static void
Viv2DFinishAccess(PixmapPtr pPixmap, int index)
{
	Viv2DPixmapPrivPtr priv = exaGetPixmapDriverPrivate(pPixmap);
//	VIV2D_DBG_MSG("Viv2DFinishAccess %d", index);

	pPixmap->devPrivate.ptr = NULL;

	/* NOTE: can we use EXA migration module to track which parts of the
	 * buffer was accessed by sw, and pass that info down to kernel to
	 * do a more precise cache flush..
	 */
	etna_bo_cpu_fini(priv->bo);
}

/**
 * PixmapIsOffscreen() is an optional driver replacement to
 * exaPixmapHasGpuCopy(). Set to NULL if you want the standard behaviour
 * of exaPixmapHasGpuCopy().
 *
 * @param pPix the pixmap
 * @return TRUE if the given drawable is in framebuffer memory.
 *
 * exaPixmapHasGpuCopy() is used to determine if a pixmap is in offscreen
 * memory, meaning that acceleration could probably be done to it, and that it
 * will need to be wrapped by PrepareAccess()/FinishAccess() when accessing it
 * with the CPU.
 */
static Bool
Viv2DPixmapIsOffscreen(PixmapPtr pPixmap)
{
	/* offscreen means in 'gpu accessible memory', not that it's off the
	 * visible screen.  We currently have no special constraints, since
	 * OMAP has a flat memory model (no separate GPU memory).  If
	 * individual EXA implementation has additional constraints, like
	 * buffer size or mapping in GPU MMU, it should wrap this function.
	 */
	Viv2DPixmapPrivPtr priv = exaGetPixmapDriverPrivate(pPixmap);
	return priv && priv->bo;
}

static Bool
CloseScreen(CLOSE_SCREEN_ARGS_DECL)
{
#if 0 // TODO need to change CloseScreen/FreeScreen ..
	exaDriverFini(pScreen);
	free(pNv->EXADriverPtr);
#endif
	return TRUE;
}

static void
FreeScreen(FREE_SCREEN_ARGS_DECL)
{
}

void *AttachViv2DDMABuf(OMAPEXAPtr omap_exa, int fd) {
	Viv2DEXAPtr v2d_exa = (Viv2DEXAPtr)omap_exa;
	Viv2DPtr v2d = v2d_exa->v2d;
	v2d->bo = etna_bo_from_dmabuf(v2d->dev, fd);
	return NULL;
}


OMAPEXAPtr
InitViv2DEXA(ScreenPtr pScreen, ScrnInfoPtr pScrn, int fd)
{
	Viv2DEXAPtr v2d_exa = calloc(sizeof (*v2d_exa), 1);
	OMAPEXAPtr omap_exa = (OMAPEXAPtr)v2d_exa;
	ExaDriverPtr exa;
	Viv2DPtr v2d = calloc(sizeof (*v2d), 1);
	int etnavivFD;
	uint64_t model, revision;

	etnavivFD = VIV2DDetectDevice("etnaviv");

	if (etnavivFD) {
		INFO_MSG("Viv2DEXA: Etnaviv driver found");
	} else {
		goto fail;
	}

	v2d->fd = etnavivFD;

	v2d->dev = etna_device_new(etnavivFD);
	if (!v2d->dev) {
		ERROR_MSG("Viv2DEXA: Failed to load device");
		goto fail;
	}

	v2d->gpu = etna_gpu_new(v2d->dev, 0);
	if (!v2d->gpu) {
		ERROR_MSG("Viv2DEXA: Failed to create gpu");
		goto fail;
	}
	etna_gpu_get_param(v2d->gpu, ETNA_GPU_MODEL, &model);
	etna_gpu_get_param(v2d->gpu, ETNA_GPU_REVISION, &revision);
	INFO_MSG("Viv2DEXA: Vivante GC%x GPU revision %x found !", (uint32_t)model, (uint32_t)revision);

	v2d->pipe = etna_pipe_new(v2d->gpu, ETNA_PIPE_2D);
	if (!v2d->pipe) {
		ERROR_MSG("Viv2DEXA: Failed to create pipe");
		goto fail;
	}

	v2d->stream = etna_cmd_stream_new(v2d->pipe, VIV2D_STREAM_SIZE, NULL, NULL);
	if (!v2d->stream) {
		ERROR_MSG("Viv2DEXA: Failed to create stream");
		goto fail;
	}

	v2d->bo = NULL;

	v2d_exa->v2d = v2d;

	exa = exaDriverAlloc();
	if (!exa) {
		goto fail;
	}

	v2d_exa->exa = exa;

	exa->exa_major = EXA_VERSION_MAJOR;
	exa->exa_minor = EXA_VERSION_MINOR;

	exa->pixmapOffsetAlign = 0;
	exa->pixmapPitchAlign = 32 * 4; // see OMAPCalculateStride()
	exa->flags = EXA_OFFSCREEN_PIXMAPS |
	             EXA_HANDLES_PIXMAPS | EXA_SUPPORTS_PREPARE_AUX;
	exa->maxX = 4096;
	exa->maxY = 4096;

	/* Required EXA functions: */
	exa->WaitMarker = Viv2DWaitMarker;
	exa->CreatePixmap2 = Viv2DCreatePixmap;
	exa->DestroyPixmap = Viv2DDestroyPixmap;
	exa->ModifyPixmapHeader = Viv2DModifyPixmapHeader;

	exa->PrepareAccess = Viv2DPrepareAccess;
	exa->FinishAccess = Viv2DFinishAccess;
	exa->PixmapIsOffscreen = Viv2DPixmapIsOffscreen;

#ifdef VIV2D_COPY
	exa->PrepareCopy = Viv2DPrepareCopy;
	exa->Copy = Viv2DCopy;
	exa->DoneCopy = Viv2DDoneCopy;
#else
	exa->PrepareCopy = PrepareCopyFail;
#endif

#ifdef VIV2D_SOLID
	exa->PrepareSolid = Viv2DPrepareSolid;
	exa->Solid = Viv2DSolid;
	exa->DoneSolid = Viv2DDoneSolid;
#else
	exa->PrepareSolid = PrepareSolidFail;
#endif

#ifdef VIV2D_COMPOSITE
	exa->CheckComposite = Viv2DCheckComposite;
	exa->PrepareComposite = Viv2DPrepareComposite;
	exa->Composite = Viv2DComposite;
	exa->DoneComposite = Viv2DDoneComposite;
#else
	exa->CheckComposite = CheckCompositeFail;
	exa->PrepareComposite = PrepareCompositeFail;
#endif

#ifdef VIV2D_UPLOAD_TO_SCREEN
	exa->UploadToScreen = Viv2DUploadToScreen;
#endif

	if (! exaDriverInit(pScreen, exa)) {
		ERROR_MSG("exaDriverInit failed");
		goto fail;
	}

	omap_exa->CloseScreen = CloseScreen;
	omap_exa->FreeScreen = FreeScreen;

	return omap_exa;

fail:
	if (v2d_exa) {
		free(v2d_exa);
	}
	return NULL;
}

