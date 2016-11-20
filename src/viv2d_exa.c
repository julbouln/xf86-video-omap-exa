
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

#include "state.xml.h"
#include "state_2d.xml.h"
#include "cmdstream.xml.h"

#include "exa.h"

#define VIV2D_SOLID 1
#define VIV2D_COPY 1
//#define VIV2D_COMPOSITE 1

//#define VIV2D_MIN_HW_HEIGHT 64
//#define VIV2D_MIN_HW_SIZE_24BIT (256 * 256)

#define VIV2D_MIN_HW_HEIGHT 64
#define VIV2D_MIN_HW_SIZE_24BIT (64 * 64)

//#define VIV2D_MIN_HW_HEIGHT 16
//#define VIV2D_MIN_HW_SIZE_24BIT (16 * 16)

#define VIV2D_MAX_RECTS 256

#define VIV2D_MSG(fmt, ...)
/*#define VIV2D_MSG(fmt, ...) \
		do { xf86Msg(X_INFO, fmt "\n",\
				##__VA_ARGS__); } while (0)
*/
#define VIV2D_FAIL_MSG(fmt, ...)
/*#define VIV2D_FAIL_MSG(fmt, ...) \
		do { xf86Msg(X_WARNING, fmt "\n",\
				##__VA_ARGS__); } while (0)
*/

typedef struct _Viv2DRect {
	int x1;
	int y1;
	int x2;
	int y2;
} Viv2DRect;

#define STATE_IDLE 0
#define STATE_WORKING 1
#define STATE_PENDING 2

typedef struct {
	void *priv;			/* EXA submodule private data */
	struct etna_bo *bo;
	int state;
	int width;
	int height;
	int pitch;
	Bool tiled;
} Viv2DPixmapPrivRec, *Viv2DPixmapPrivPtr;

typedef struct _Viv2DSolidOp {
	int rop;
	uint32_t fg;
	uint32_t mask;
	int cur_rect;
	Viv2DRect rects[VIV2D_MAX_RECTS];
} Viv2DSolidOp;

typedef struct _Viv2DCopyOp {
	int rop;
	uint32_t mask;
	int src_fmt;
	int dst_fmt;
	Viv2DPixmapPrivPtr src;
	Viv2DPixmapPrivPtr dst;
} Viv2DCopyOp;


/*
Raster operation foreground and background codes. Even though ROP is not used in `CLEAR`,
`HOR_FILTER_BLT`, `VER_FILTER_BLT` and alpha-enabled `BIT_BLTs`, ROP code still has to be
programmed, because the engine makes the decision whether source, destination and pattern are
involved in the current operation and the correct decision is essential for the engine to complete
the operation as expected.

ROP builds a lookup table for a logical operation with 2, 3 or 4 inputs (depending on ROP type). So
for a ROP3, for example, the ROP pattern will be 2^3=8 bits.

These are the input bit for the ROPs, per ROP type:

`ROP2_PATTERN` [untested]
    bit 0 destination
    bit 1 pattern

`ROP2_SOURCE` [untested]
    bit 0 destination
    bit 1 source

`ROP3` (uses `ROP_FG` only)
    bit 0 destination
    bit 1 source
    bit 2 pattern

`ROP4` (uses `ROP_FG` and `ROP_BG`)
    bit 0 destination
    bit 1 source
    bit 2 pattern
    bit "3" foreground/background (`ROP_FG` / `ROP_BG`)

*/

#define ROP_BLACK 				0x00
#define ROP_NOT_SRC_AND_NOT_DST 0x11
#define ROP_NOT_SRC_AND_DST 	0x22
#define ROP_NOT_SRC				0x33
#define ROP_SRC_AND_NOT_DST 	0x44
#define ROP_NOT_DST				0x55
#define ROP_DST_XOR_SRC 		0x66
#define ROP_NOT_SRC_OR_NOT_DST	0x77
#define ROP_DST_AND_SRC 		0x88
#define ROP_NOT_SRC_XOR_DST		0x99
#define ROP_DST 				0xaa
#define ROP_NOT_SRC_OR_DST		0xbb
#define ROP_SRC 				0xcc
#define ROP_SRC_OR_NOT_DST		0xdd
#define ROP_DST_OR_SRC 			0xee
#define ROP_WHITE 				0xff

typedef struct _Viv2DBlendOp {
	int op;
	int srcBlendMode;
	int dstBlendMode;
} Viv2DBlendOp;

static const Viv2DBlendOp viv2d_blend_op[] = {
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

typedef struct _Viv2DPictFormat {
	int exaFmt;
	int bpp;
	unsigned int fmt;
	int alphaBits;
} Viv2DPictFormat;

typedef struct _Viv2DCompositeOp {
	Viv2DBlendOp blendOp;
	Viv2DPictFormat srcFmt;
	Viv2DPictFormat dstFmt;

	Viv2DPixmapPrivPtr src;
	Viv2DPixmapPrivPtr dst;

} Viv2DCompositeOp;

#define NO_PICT_FORMAT -1


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
/**
 * Picture Formats and their counter parts
 */
#define VIV2D_PICT_FORMAT_COUNT 8
static const Viv2DPictFormat
viv2d_pict_format[] = {
	{PICT_a8r8g8b8, 32, DE_FORMAT_A8R8G8B8, 8},
	{PICT_x8r8g8b8, 32, DE_FORMAT_X8R8G8B8, 0},
	{PICT_r5g6b5, 16, DE_FORMAT_R5G6B5, 0},
	{PICT_a1r5g5b5, 16, DE_FORMAT_A1R5G5B5, 1},
	{PICT_x1r5g5b5, 16, DE_FORMAT_X1R5G5B5, 0},
	{PICT_a4r4g4b4, 16, DE_FORMAT_A4R4G4B4, 4},
	{PICT_x4r4g4b4, 16, DE_FORMAT_X4R4G4B4, 0},
//	{PICT_a8, 16, DE_FORMAT_A8, 0},
	{NO_PICT_FORMAT, 0, 0, 0}
	/*END*/
};

typedef struct _Viv2DRec {
	struct etna_device *dev;
	struct etna_gpu *gpu;
	struct etna_pipe *pipe;
	struct etna_cmd_stream *stream;

	Viv2DSolidOp *solid_op;
	Viv2DCopyOp *copy_op;
	Viv2DCompositeOp *comp_op;

	struct etna_bo *bo;
	int width;
	int height;

} Viv2DRec, *Viv2DPtr;

typedef struct {
	OMAPEXARec base;
	ExaDriverPtr exa;
	/* add any other driver private data here.. */
	Viv2DPtr v2d;
} Viv2DEXARec, *Viv2DEXAPtr;

// utils

static inline void etna_emit_load_state(struct etna_cmd_stream *stream,
                                        const uint16_t offset, const uint16_t count)
{
	uint32_t v;

	v = 	(VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | VIV_FE_LOAD_STATE_HEADER_OFFSET(offset) |
	         (VIV_FE_LOAD_STATE_HEADER_COUNT(count) & VIV_FE_LOAD_STATE_HEADER_COUNT__MASK));

	etna_cmd_stream_emit(stream, v);
}

static inline void etna_set_state(struct etna_cmd_stream *stream, uint32_t address, uint32_t value)
{
	etna_cmd_stream_reserve(stream, 2);
	etna_emit_load_state(stream, address >> 2, 1);
	etna_cmd_stream_emit(stream, value);
}

static inline void etna_set_state_from_bo(struct etna_cmd_stream *stream,
        uint32_t address, struct etna_bo *bo)
{
	etna_cmd_stream_reserve(stream, 2);
	etna_emit_load_state(stream, address >> 2, 1);

	etna_cmd_stream_reloc(stream, &(struct etna_reloc) {
		.bo = bo,
		 .flags = ETNA_RELOC_READ,
		  .offset = 0,
	});
}

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

			if (rc == 0)
				return fd;
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


static inline Bool Viv2DFormat(unsigned int depth, unsigned int bpp, uint32_t *fmt)
{
	switch (bpp) {
	case 16:
		if (depth == 15)
			*fmt = DE_FORMAT_A1R5G5B5;
		else
			*fmt = DE_FORMAT_R5G6B5;
		return TRUE;
		break;
	case 32:
		*fmt = DE_FORMAT_A8R8G8B8;
		return TRUE;
		break;
	default:
		break;
	}
	return FALSE;
}

static Viv2DRec*
Viv2DPrivFromPixmap(PixmapPtr pPixmap)
{
	ScrnInfoPtr pScrn = pix2scrn(pPixmap);
	OMAPPtr pOMAP = OMAPPTR(pScrn);
	Viv2DEXAPtr exa = (Viv2DEXAPtr)(pOMAP->pOMAPEXA);
	Viv2DRec *v2d = exa->v2d;
	return v2d;
}


static void *
Viv2DCreatePixmap (ScreenPtr pScreen, int width, int height,
                   int depth, int usage_hint, int bitsPerPixel,
                   int *new_fb_pitch)
{
	Viv2DPixmapPrivPtr priv = calloc(sizeof(Viv2DPixmapPrivRec), 1);
	return priv;
}

static void
Viv2DDestroyPixmap(ScreenPtr pScreen, void *driverPriv)
{
	Viv2DPixmapPrivPtr priv = driverPriv;
	Viv2DRec *v2d;
	Viv2DEXAPtr exa;
	OMAPPtr pOMAP = OMAPPTR_FROM_SCREEN(pScreen);
	exa = (Viv2DEXAPtr)(pOMAP->pOMAPEXA);
	v2d = exa->v2d;

//	VIV2D_MSG("Viv2DDestroyPixmap");

	if (priv->bo && v2d->bo != priv->bo) {
//		VIV2D_MSG("Viv2DDestroyPixmap freeing bo %p/%p", driverPriv, etna_bo_map(priv->bo));
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

//	VIV2D_MSG("Viv2DModifyPixmapHeader %p <=> %p", pPixData, etna_bo_map(v2d->bo));

	priv->width = width	= pPixmap->drawable.width;
	priv->height = height	= pPixmap->drawable.height;
	depth	= pPixmap->drawable.depth;
	bitsPerPixel = pPixmap->drawable.bitsPerPixel;

//	pPixmap->devKind = Viv2DPitch(width, bitsPerPixel);
	priv->pitch = pPixmap->devKind = OMAPCalculateStride(width, bitsPerPixel);

	if (pPixData == omap_bo_map(pOMAP->scanout)) {
		VIV2D_MSG("Viv2DModifyPixmapHeader wrapping scanout buffer pPixData:%p etna:%p omap:%p", pPixData, etna_bo_map(v2d->bo), omap_bo_map(pOMAP->scanout));
		priv->bo = v2d->bo;
		return TRUE;
	} else

		if (pPixData) {
			/* we can't accelerate this pixmap, and don't ever want to
			 * see it again..
			 */
			pPixmap->devPrivate.ptr = pPixData;
			pPixmap->devKind = devKind;

			/* scratch-pixmap (see GetScratchPixmapHeader()) gets recycled,
			 * so could have a previous bo!
			 */
			if (priv->bo) {
//			VIV2D_MSG("Viv2DModifyPixmapHeader freeing bo %p/%p", priv, etna_bo_map(priv->bo));
				etna_bo_del(priv->bo);
			}
			priv->bo = NULL;

			return FALSE;
		}
//	VIV2D_MSG("Viv2DModifyPixmapHeader %dx%d %d %p",
//	         width, height, depth,
//	         pPixData);
//	VIV2D_MSG("Viv2DModifyPixmapHeader(2) %dx%d %d",
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
		priv->state = STATE_IDLE;
		priv->tiled = FALSE;
	}

	if (!priv->bo) {
		ERROR_MSG("failed to allocate %dx%d bo, size=%d, flags=%08x",
		          width, height, size, flags);
	}

	return priv->bo != NULL;
}

static inline uint32_t viv2d_scale16(uint32_t val, int bits)
{
	val <<= (16 - bits);
	while (bits < 16) {
		val |= val >> bits;
		bits <<= 1;
	}
	return val >> 8;
}

static inline uint32_t viv2d_colour(Pixel pixel, int depth) {
	uint32_t colour;
	switch (depth) {
	case 15: /* A1R5G5B5 */
		colour = (pixel & 0x8000 ? 0xff000000 : 0) |
		         viv2d_scale16((pixel & 0x7c00) >> 10, 5) << 16 |
		         viv2d_scale16((pixel & 0x03e0) >> 5, 5) << 8 |
		         viv2d_scale16((pixel & 0x001f), 5);
		break;
	case 16: /* R5G6B5 */
		colour = 0xff000000 |
		         viv2d_scale16((pixel & 0xf800) >> 11, 5) << 16 |
		         viv2d_scale16((pixel & 0x07e0) >> 5, 6) << 8 |
		         viv2d_scale16((pixel & 0x001f), 5);
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
	uint32_t fmt;
	uint32_t pitch;

	Viv2DSolidOp *solidOp;

	if (alu != GXcopy)
		return FALSE;

	if (!EXA_PM_IS_SOLID(&pPixmap->drawable, planemask))
		return FALSE;

	if (pPixmap->drawable.height < VIV2D_MIN_HW_HEIGHT || pPixmap->drawable.width * pPixmap->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_FAIL_MSG("Viv2DPrepareSolid dest drawable is too small %dx%d", pPixmap->drawable.width, pPixmap->drawable.height);
		return FALSE;
	}

	if (!Viv2DFormat(pPixmap->drawable.depth, pPixmap->drawable.bitsPerPixel, &fmt)) {
		VIV2D_FAIL_MSG("Viv2DPrepareSolid unsupported format for depth:%d bpp:%d", pPixmap->drawable.depth, pPixmap->drawable.bitsPerPixel);
		return FALSE;
	}

	pitch = pPixmap->devKind;

	solidOp = calloc(sizeof(*solidOp), 1);
	solidOp->fg = viv2d_colour(fg, pPixmap->drawable.depth);

	solidOp->mask = (uint32_t)planemask;


	VIV2D_MSG("Viv2DPrepareSolid %p %dx%d, %x %x %d, %d %d %p", dst,
	          pPixmap->drawable.width, pPixmap->drawable.height, solidOp->fg ,
	          solidOp->mask, pPixmap->drawable.depth, alu, pitch, dst);

	solidOp->cur_rect = 0;
	v2d->solid_op = solidOp;

	dst->state = STATE_WORKING;

	etna_set_state_from_bo(v2d->stream, VIVS_DE_DEST_ADDRESS, dst->bo);
	etna_set_state(v2d->stream, VIVS_DE_DEST_STRIDE, pitch);

	etna_set_state(v2d->stream, VIVS_DE_DEST_ROTATION_CONFIG, 0);
	etna_set_state(v2d->stream, VIVS_DE_DEST_CONFIG,
	               VIVS_DE_DEST_CONFIG_FORMAT(fmt) |
	               VIVS_DE_DEST_CONFIG_SWIZZLE(DE_SWIZZLE_ARGB) |
	               VIVS_DE_DEST_CONFIG_COMMAND_CLEAR |
	               VIVS_DE_DEST_CONFIG_TILED_DISABLE |
	               VIVS_DE_DEST_CONFIG_MINOR_TILED_DISABLE
	              );

	etna_set_state(v2d->stream, VIVS_DE_ROP,
	               VIVS_DE_ROP_ROP_FG(ROP_SRC) | VIVS_DE_ROP_ROP_BG(ROP_SRC) | VIVS_DE_ROP_TYPE_ROP4);
	etna_set_state(v2d->stream, VIVS_DE_CLIP_TOP_LEFT,
	               VIVS_DE_CLIP_TOP_LEFT_X(0) |
	               VIVS_DE_CLIP_TOP_LEFT_Y(0)
	              );
	etna_set_state(v2d->stream, VIVS_DE_CLIP_BOTTOM_RIGHT,
	               VIVS_DE_CLIP_BOTTOM_RIGHT_X(pPixmap->drawable.width) |
	               VIVS_DE_CLIP_BOTTOM_RIGHT_Y(pPixmap->drawable.height)
	              );

	/* Clear color PE20 */
	etna_set_state(v2d->stream, VIVS_DE_CLEAR_PIXEL_VALUE32, solidOp->fg );
	/* Clear color PE10 */
	etna_set_state(v2d->stream, VIVS_DE_CLEAR_BYTE_MASK, 0xff);
	etna_set_state(v2d->stream, VIVS_DE_CLEAR_PIXEL_VALUE_LOW, solidOp->fg);
	etna_set_state(v2d->stream, VIVS_DE_CLEAR_PIXEL_VALUE_HIGH, solidOp->fg);

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
//	ScrnInfoPtr pScrn = pix2scrn(pPixmap);
	Viv2DSolidOp *solidOp = v2d->solid_op;
	Viv2DRect rect;

//	VIV2D_MSG("Viv2DSolid %dx%d:%dx%d %d", x1, y1, x2, y2, solidOp->cur_rect);

	rect.x1 = x1;
	rect.y1 = y1;
	rect.x2 = x2;
	rect.y2 = y2;
	solidOp->rects[solidOp->cur_rect] = rect;
	solidOp->cur_rect++;
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
	Viv2DSolidOp *solidOp = v2d->solid_op;
	int i;

	VIV2D_MSG("Viv2DDoneSolid %d", solidOp->cur_rect);

	etna_cmd_stream_reserve(v2d->stream, solidOp->cur_rect * 2 + 2);
	etna_cmd_stream_emit(v2d->stream,
	                     VIV_FE_DRAW_2D_HEADER_OP_DRAW_2D | VIV_FE_DRAW_2D_HEADER_COUNT(solidOp->cur_rect) /* render one rectangle */
	                    );
	etna_cmd_stream_emit(v2d->stream, 0x0); /* rectangles start aligned */

	for (i = 0; i < solidOp->cur_rect; i++) {
		Viv2DRect rect = solidOp->rects[i];
		etna_cmd_stream_emit(v2d->stream, VIV_FE_DRAW_2D_TOP_LEFT_X(rect.x1) |
		                     VIV_FE_DRAW_2D_TOP_LEFT_Y(rect.y1));
		etna_cmd_stream_emit(v2d->stream, VIV_FE_DRAW_2D_BOTTOM_RIGHT_X(rect.x2) |
		                     VIV_FE_DRAW_2D_BOTTOM_RIGHT_Y(rect.y2));
	}

	etna_set_state(v2d->stream, 1, 0);
	etna_set_state(v2d->stream, 1, 0);
	etna_set_state(v2d->stream, 1, 0);

	etna_set_state(v2d->stream, VIVS_GL_FLUSH_CACHE, VIVS_GL_FLUSH_CACHE_PE2D);

	dst->state = STATE_PENDING;

	free(solidOp);
	v2d->solid_op = NULL;
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
//	ScrnInfoPtr pScrn = pix2scrn(pDstPixmap);
	uint32_t src_fmt, dst_fmt;

	Viv2DCopyOp *copyOp;

	if (alu != GXcopy) {
		VIV2D_FAIL_MSG("Viv2DPrepareCopy alu is not GXcopy %d", alu);
		return FALSE;
	}

	if (pDstPixmap->drawable.height < VIV2D_MIN_HW_HEIGHT || pDstPixmap->drawable.width * pDstPixmap->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_FAIL_MSG("Viv2DPrepareCopy dest drawable is too small %dx%d", pDstPixmap->drawable.width, pDstPixmap->drawable.height);
		return FALSE;
	}

	if (!Viv2DFormat(pSrcPixmap->drawable.depth, pSrcPixmap->drawable.bitsPerPixel, &src_fmt)) {
		VIV2D_FAIL_MSG("Viv2DPrepareCopy unsupported format for depth:%d bpp:%d", pSrcPixmap->drawable.depth, pSrcPixmap->drawable.bitsPerPixel);
		return FALSE;
	}
	if (!Viv2DFormat(pDstPixmap->drawable.depth, pDstPixmap->drawable.bitsPerPixel, &dst_fmt)) {
		VIV2D_FAIL_MSG("Viv2DPrepareCopy unsupported format for depth:%d bpp:%d", pDstPixmap->drawable.depth, pDstPixmap->drawable.bitsPerPixel);
		return FALSE;
	}

	copyOp = calloc(sizeof(*copyOp), 1);

	copyOp->mask = (uint32_t)planemask;

	VIV2D_MSG("Viv2DPrepareCopy %p(%dx%d) -> %p(%dx%d) %dx%d %d %x",
	          src, src->width, src->height,
	          dst, dst->width, dst->height,
	          dx, dy, alu, (uint32_t)planemask);

	copyOp->src = src;
	copyOp->dst = dst;

	copyOp->src_fmt = src_fmt;
	copyOp->dst_fmt = dst_fmt;

	v2d->copy_op = copyOp;

//	src->state = STATE_WORKING;
	dst->state = STATE_WORKING;

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
//	ScrnInfoPtr pScrn = pix2scrn(pDstPixmap);
	Viv2DCopyOp *copyOp = v2d->copy_op;

	VIV2D_MSG("Viv2DCopy %p(%dx%d) -> %p(%dx%d) : %dx%d", copyOp->src, srcX, srcY, copyOp->dst, dstX, dstY, width, height);

	etna_set_state_from_bo(v2d->stream, VIVS_DE_SRC_ADDRESS, copyOp->src->bo);
	etna_set_state(v2d->stream, VIVS_DE_SRC_STRIDE, copyOp->src->pitch);
	etna_set_state(v2d->stream, VIVS_DE_SRC_ROTATION_CONFIG, 0);
	etna_set_state(v2d->stream, VIVS_DE_SRC_CONFIG,
	               VIVS_DE_SRC_CONFIG_SOURCE_FORMAT(copyOp->src_fmt) |
	               VIVS_DE_SRC_CONFIG_SWIZZLE(DE_SWIZZLE_ARGB) |
	               VIVS_DE_SRC_CONFIG_LOCATION_MEMORY |
	               VIVS_DE_SRC_CONFIG_PE10_SOURCE_FORMAT(copyOp->src_fmt));
	etna_set_state(v2d->stream, VIVS_DE_SRC_ORIGIN,
	               VIVS_DE_SRC_ORIGIN_X(srcX) |
	               VIVS_DE_SRC_ORIGIN_Y(srcY));
	etna_set_state(v2d->stream, VIVS_DE_SRC_SIZE,
	               VIVS_DE_SRC_SIZE_X(width) |
	               VIVS_DE_SRC_SIZE_Y(height)
	              ); // source size is ignored
//	etna_set_state(v2d->stream, VIVS_DE_SRC_COLOR_BG, 0xff303030);
//	etna_set_state(v2d->stream, VIVS_DE_SRC_COLOR_FG, 0xff12ff56);
//	etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_LOW, 0);
//	etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_HIGH, 0);
	etna_set_state_from_bo(v2d->stream, VIVS_DE_DEST_ADDRESS, copyOp->dst->bo);
	etna_set_state(v2d->stream, VIVS_DE_DEST_STRIDE, pDstPixmap->devKind);
	etna_set_state(v2d->stream, VIVS_DE_DEST_ROTATION_CONFIG, 0);
	etna_set_state(v2d->stream, VIVS_DE_DEST_CONFIG,
	               VIVS_DE_DEST_CONFIG_FORMAT(copyOp->dst_fmt) |
	               VIVS_DE_DEST_CONFIG_SWIZZLE(DE_SWIZZLE_ARGB) |
	               VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT |
	               VIVS_DE_DEST_CONFIG_TILED_DISABLE |
	               VIVS_DE_DEST_CONFIG_MINOR_TILED_DISABLE
	              );
	etna_set_state(v2d->stream, VIVS_DE_ROP,
	               VIVS_DE_ROP_ROP_FG(ROP_SRC) | VIVS_DE_ROP_ROP_BG(ROP_SRC) | VIVS_DE_ROP_TYPE_ROP4);
	etna_set_state(v2d->stream, VIVS_DE_CLIP_TOP_LEFT,
	               VIVS_DE_CLIP_TOP_LEFT_X(0) |
	               VIVS_DE_CLIP_TOP_LEFT_Y(0)
	              );
	etna_set_state(v2d->stream, VIVS_DE_CLIP_BOTTOM_RIGHT,
	               VIVS_DE_CLIP_BOTTOM_RIGHT_X(copyOp->dst->width) |
	               VIVS_DE_CLIP_BOTTOM_RIGHT_Y(copyOp->dst->height)
	              );

	etna_set_state(v2d->stream, VIVS_DE_ALPHA_CONTROL,
	               VIVS_DE_ALPHA_CONTROL_ENABLE_OFF);


	etna_cmd_stream_reserve(v2d->stream, 2 + 2);
	etna_cmd_stream_emit(v2d->stream,
	                     VIV_FE_DRAW_2D_HEADER_OP_DRAW_2D |
	                     VIV_FE_DRAW_2D_HEADER_COUNT(1) |
	                     VIV_FE_DRAW_2D_HEADER_DATA_COUNT(0)
	                    );
	etna_cmd_stream_emit(v2d->stream, 0x0); /* rectangles start aligned */

	etna_cmd_stream_emit(v2d->stream, VIV_FE_DRAW_2D_TOP_LEFT_X(dstX) |
	                     VIV_FE_DRAW_2D_TOP_LEFT_Y(dstY));
	etna_cmd_stream_emit(v2d->stream, VIV_FE_DRAW_2D_BOTTOM_RIGHT_X(dstX + width) |
	                     VIV_FE_DRAW_2D_BOTTOM_RIGHT_Y(dstY + height));

	etna_set_state(v2d->stream, 1, 0);
	etna_set_state(v2d->stream, 1, 0);
	etna_set_state(v2d->stream, 1, 0);

	etna_set_state(v2d->stream, VIVS_GL_FLUSH_CACHE, VIVS_GL_FLUSH_CACHE_PE2D);

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
//	ScrnInfoPtr pScrn = pix2scrn(pDstPixmap);
	Viv2DCopyOp *copyOp = v2d->copy_op;

//	VIV2D_MSG("Viv2DDoneCopy %d", copyOp->cur_rect);

//	etna_cmd_stream_finish(v2d->stream);
	dst->state = STATE_PENDING;

	free(copyOp);
	v2d->copy_op = NULL;
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

static Bool Viv2DGetPictureFormat(int exa_fmt, Viv2DPictFormat *fmt) {
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

static inline int IsSourceAlphaRequired(int op) {
	return (((
	             (1 << PictOpOver) |
	             (1 << PictOpInReverse) |
	             (1 << PictOpOutReverse) |
	             (1 << PictOpAtop) |
	             (1 << PictOpAtopReverse) |
	             (1 << PictOpXor) |
	             0) >> (op)) & 1);
}

static inline int IsDestAlphaRequired(int op) {
	return (((
	             (1 << PictOpOverReverse) |
	             (1 << PictOpIn) |
	             (1 << PictOpOut) |
	             (1 << PictOpAtop) |
	             (1 << PictOpAtopReverse) |
	             (1 << PictOpXor) |
	             0) >> (op)) & 1);
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
	Viv2DCompositeOp compOp;

	if (pDst == NULL) {
		return FALSE;
	}

	if (pSrc == NULL) {
		return FALSE;
	}

	VIV2D_MSG("Viv2DCheckComposite %d : %dx%d(%s) -> %dx%d(%s)",
	          op, pSrc->drawable.width, pSrc->drawable.height, pix_format_name(pSrcPicture->format),
	          pDst->drawable.width, pDst->drawable.height, pix_format_name(pDstPicture->format)
	         );

	if (pDst->drawable.height < VIV2D_MIN_HW_HEIGHT || pDst->drawable.width * pDst->drawable.height < VIV2D_MIN_HW_SIZE_24BIT) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite dest drawable is too small %dx%d", pDst->drawable.width, pDst->drawable.height);
		return FALSE;
	}

	/*For forward compatibility*/
	if (op > PictOpSaturate) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite op unsupported");
		return FALSE;
	}

	/*Format Checks*/
	if (!Viv2DGetPictureFormat(pSrcPicture->format, &compOp.srcFmt)) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite unsupported src format %s", pix_format_name(pSrcPicture->format));
		return FALSE;
	}

	if (!Viv2DGetPictureFormat(pDstPicture->format, &compOp.dstFmt)) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite unsupported dst format %s", pix_format_name(pDstPicture->format));
		return FALSE;
	}

	if ( pSrc->drawable.bitsPerPixel < 8 || pDst->drawable.bitsPerPixel < 8 )     {
		VIV2D_FAIL_MSG("Viv2DCheckComposite unsupported 8bits format");
		return FALSE;
	}

	/*No Gradient*/
	if (pSrcPicture->pSourcePict) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite gradiant unsupported");
		return FALSE;
	}

	/*
		if (pMaskPicture && pMaskPicture->transform) {
			return FALSE;
		}

		if (pMaskPicture && pMaskPicture->componentAlpha) {
			return FALSE;
		}

		if (pMaskPicture && pMaskPicture->repeat) {
			return FALSE;
		}
	*/

	if ( pSrcPicture->transform ) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite transform unsupported");
		return FALSE;
	}

	if ( pSrcPicture->repeat ) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite repeat unsupported");
		return FALSE;
	}

	if (IsSourceAlphaRequired(op)) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite alpha src unsupported");
		return FALSE;
	}

	if (IsDestAlphaRequired(op)) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite alpha src unsupported");
		return FALSE;
	}

	if ((pMaskPicture != NULL)) {
		VIV2D_FAIL_MSG("Viv2DCheckComposite mask unsupported");
		return FALSE;
	} else {

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
Viv2DPrepareComposite(int op, PicturePtr pSrcPicture,
                      PicturePtr pMaskPicture,
                      PicturePtr pDstPicture,
                      PixmapPtr pSrc, PixmapPtr pMask, PixmapPtr pDst) {
	Viv2DPixmapPrivPtr src = exaGetPixmapDriverPrivate(pSrc);
	Viv2DPixmapPrivPtr dst = exaGetPixmapDriverPrivate(pDst);
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDst);
	Viv2DCompositeOp *compOp;

//	VIV2D_MSG("Viv2DPrepareComposite %d", op);

	compOp = calloc(sizeof(*compOp), 1);

	compOp->blendOp = viv2d_blend_op[op];

	if (!Viv2DGetPictureFormat(pSrcPicture->format, &compOp->srcFmt)) {
		VIV2D_FAIL_MSG("Viv2DPrepareComposite unsupported src format %s", pix_format_name(pSrcPicture->format));
		return FALSE;
	}

	if (!Viv2DGetPictureFormat(pDstPicture->format, &compOp->dstFmt)) {
		VIV2D_FAIL_MSG("Viv2DPrepareComposite unsupported dst format %s", pix_format_name(pSrcPicture->format));
		return FALSE;
	}

	compOp->src = src;
	compOp->dst = dst;

	v2d->comp_op = compOp;

//	src->state = STATE_WORKING;
	dst->state = STATE_WORKING;

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
static void
Viv2DComposite(PixmapPtr pDst, int srcX, int srcY, int maskX, int maskY,
               int dstX, int dstY, int width, int height) {
	Viv2DRec *v2d = Viv2DPrivFromPixmap(pDst);
//	ScrnInfoPtr pScrn = pix2scrn(pDstPixmap);
	Viv2DCompositeOp *compOp = v2d->comp_op;

	VIV2D_MSG("Viv2DComposite %dx%d -> %dx%d : %dx%d", srcX, srcY, dstX, dstY, width, height);

	etna_set_state_from_bo(v2d->stream, VIVS_DE_SRC_ADDRESS, compOp->src->bo);
	etna_set_state(v2d->stream, VIVS_DE_SRC_STRIDE, compOp->src->pitch);
	etna_set_state(v2d->stream, VIVS_DE_SRC_ROTATION_CONFIG, 0);
	etna_set_state(v2d->stream, VIVS_DE_SRC_CONFIG,
	               VIVS_DE_SRC_CONFIG_SOURCE_FORMAT(compOp->srcFmt.fmt) |
	               VIVS_DE_SRC_CONFIG_SWIZZLE(DE_SWIZZLE_ARGB) |
	               VIVS_DE_SRC_CONFIG_LOCATION_MEMORY |
	               VIVS_DE_SRC_CONFIG_PACK_PACKED8 |
//	               VIVS_DE_SRC_CONFIG_TILED_ENABLE |
	               VIVS_DE_SRC_CONFIG_PE10_SOURCE_FORMAT(compOp->srcFmt.fmt));
	etna_set_state(v2d->stream, VIVS_DE_SRC_ORIGIN,
	               VIVS_DE_SRC_ORIGIN_X(srcX) |
	               VIVS_DE_SRC_ORIGIN_Y(srcY));
	etna_set_state(v2d->stream, VIVS_DE_SRC_SIZE,
	               VIVS_DE_SRC_SIZE_X(compOp->src->width) |
	               VIVS_DE_SRC_SIZE_Y(compOp->src->height)
	              ); // source size is ignored
//	etna_set_state(v2d->stream, VIVS_DE_SRC_COLOR_BG, 0xff303030);
//	etna_set_state(v2d->stream, VIVS_DE_SRC_COLOR_FG, 0xff12ff56);
//	etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_LOW, 0);
//	etna_set_state(v2d->stream, VIVS_DE_STRETCH_FACTOR_HIGH, 0);
	etna_set_state_from_bo(v2d->stream, VIVS_DE_DEST_ADDRESS, compOp->dst->bo);
	etna_set_state(v2d->stream, VIVS_DE_DEST_STRIDE, compOp->dst->pitch);
	etna_set_state(v2d->stream, VIVS_DE_DEST_ROTATION_CONFIG, 0);
	etna_set_state(v2d->stream, VIVS_DE_DEST_CONFIG,
	               VIVS_DE_DEST_CONFIG_FORMAT(compOp->dstFmt.fmt) |
	               VIVS_DE_DEST_CONFIG_SWIZZLE(DE_SWIZZLE_ARGB) |
	               VIVS_DE_DEST_CONFIG_COMMAND_BIT_BLT |
	               VIVS_DE_DEST_CONFIG_TILED_DISABLE |
	               VIVS_DE_DEST_CONFIG_MINOR_TILED_DISABLE
	              );
	etna_set_state(v2d->stream, VIVS_DE_ROP,
	               VIVS_DE_ROP_ROP_FG(ROP_SRC) | VIVS_DE_ROP_ROP_BG(ROP_SRC) | VIVS_DE_ROP_TYPE_ROP4);
	etna_set_state(v2d->stream, VIVS_DE_CLIP_TOP_LEFT,
	               VIVS_DE_CLIP_TOP_LEFT_X(0) |
	               VIVS_DE_CLIP_TOP_LEFT_Y(0)
	              );
	etna_set_state(v2d->stream, VIVS_DE_CLIP_BOTTOM_RIGHT,
	               VIVS_DE_CLIP_BOTTOM_RIGHT_X(width) |
	               VIVS_DE_CLIP_BOTTOM_RIGHT_Y(height)
	              );

//	etna_set_state(v2d->stream, VIVS_DE_CONFIG, 0); /* TODO */
//	etna_set_state(v2d->stream, VIVS_DE_SRC_ORIGIN_FRACTION, 0);
	etna_set_state(v2d->stream, VIVS_DE_ALPHA_CONTROL,
	               VIVS_DE_ALPHA_CONTROL_ENABLE_ON |
	               VIVS_DE_ALPHA_CONTROL_PE10_GLOBAL_SRC_ALPHA(0x00) |
	               VIVS_DE_ALPHA_CONTROL_PE10_GLOBAL_DST_ALPHA(0x00));

	etna_set_state(v2d->stream, VIVS_DE_ALPHA_MODES,
	               VIVS_DE_ALPHA_MODES_SRC_ALPHA_MODE_NORMAL |
	               VIVS_DE_ALPHA_MODES_DST_ALPHA_MODE_NORMAL |
//	               VIVS_DE_ALPHA_MODES_GLOBAL_SRC_ALPHA_MODE_NORMAL |
//	               VIVS_DE_ALPHA_MODES_GLOBAL_DST_ALPHA_MODE_NORMAL |
//	               VIVS_DE_ALPHA_MODES_PE10_SRC_COLOR_MULTIPLY_ENABLE |
//	               VIVS_DE_ALPHA_MODES_PE10_DST_COLOR_MULTIPLY_ENABLE |
//	               VIVS_DE_ALPHA_MODES_SRC_ALPHA_FACTOR_DISABLE |
	               VIVS_DE_ALPHA_MODES_SRC_BLENDING_MODE(compOp->blendOp.srcBlendMode) |
//	               VIVS_DE_ALPHA_MODES_DST_ALPHA_FACTOR_DISABLE |
	               VIVS_DE_ALPHA_MODES_DST_BLENDING_MODE(compOp->blendOp.dstBlendMode));
	etna_set_state(v2d->stream, VIVS_DE_COLOR_MULTIPLY_MODES, /* PE20 */
	               VIVS_DE_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_DISABLE |
	               VIVS_DE_COLOR_MULTIPLY_MODES_DST_PREMULTIPLY_DISABLE |
	               VIVS_DE_COLOR_MULTIPLY_MODES_SRC_GLOBAL_PREMULTIPLY_DISABLE |
	               VIVS_DE_COLOR_MULTIPLY_MODES_DST_DEMULTIPLY_DISABLE);
	/*	etna_set_state(v2d->stream, VIVS_DE_DEST_ROTATION_HEIGHT, 0);
		etna_set_state(v2d->stream, VIVS_DE_SRC_ROTATION_HEIGHT, 0);
		etna_set_state(v2d->stream, VIVS_DE_ROT_ANGLE, 0);
	*/
	etna_cmd_stream_reserve(v2d->stream, 2 + 2);
	etna_cmd_stream_emit(v2d->stream,
	                     VIV_FE_DRAW_2D_HEADER_OP_DRAW_2D |
	                     VIV_FE_DRAW_2D_HEADER_COUNT(1) |
	                     VIV_FE_DRAW_2D_HEADER_DATA_COUNT(0)
	                    );
	etna_cmd_stream_emit(v2d->stream, 0x0); /* rectangles start aligned */

	etna_cmd_stream_emit(v2d->stream, VIV_FE_DRAW_2D_TOP_LEFT_X(dstX) |
	                     VIV_FE_DRAW_2D_TOP_LEFT_Y(dstY));
	etna_cmd_stream_emit(v2d->stream, VIV_FE_DRAW_2D_BOTTOM_RIGHT_X(dstX + width) |
	                     VIV_FE_DRAW_2D_BOTTOM_RIGHT_Y(dstY + height));

	etna_set_state(v2d->stream, 1, 0);
	etna_set_state(v2d->stream, 1, 0);
	etna_set_state(v2d->stream, 1, 0);

	etna_set_state(v2d->stream, VIVS_GL_FLUSH_CACHE, VIVS_GL_FLUSH_CACHE_PE2D);

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
//	ScrnInfoPtr pScrn = pix2scrn(pDst);
	Viv2DCompositeOp *compOp = v2d->comp_op;
	int i;

	VIV2D_MSG("Viv2DDoneComposite");

//	etna_cmd_stream_finish(v2d->stream);
	dst->state = STATE_PENDING;

	free(compOp);
	v2d->comp_op = NULL;
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

	switch (priv->state) {
	case STATE_IDLE:
		break;
	case STATE_WORKING:
		// not ready
		return FALSE;
	case STATE_PENDING:
		VIV2D_MSG("Viv2DPrepareAccess flushing %d/%d", v2d->stream->offset, v2d->stream->size);
		etna_set_state(v2d->stream, VIVS_GL_FLUSH_CACHE, VIVS_GL_FLUSH_CACHE_PE2D);
		etna_cmd_stream_finish(v2d->stream);
		priv->state = STATE_IDLE;
		break;
	default:
		break;
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
//	VIV2D_MSG("Viv2DFinishAccess %d", index);

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

	v2d->stream = etna_cmd_stream_new(v2d->pipe, 1024, NULL, NULL);
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

