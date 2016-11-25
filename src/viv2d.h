#ifndef VIV2D_H
#define VIV2D_H

#include <stdint.h>
#include <xorg-server.h>
#include "xf86.h"
#include "xf86_OSproc.h"

#define VIV2D_MAX_RECTS 256

//#define ETNA_BO_FROM_HANDLE_MISSING 1

#define VIV2D_DBG_MSG(fmt, ...)
/*#define VIV2D_DBG_MSG(fmt, ...) \
		do { xf86Msg(X_INFO, fmt "\n",\
				##__VA_ARGS__); } while (0)
*/

#define VIV2D_UNSUPPORTED_MSG(fmt, ...)
/*#define VIV2D_UNSUPPORTED_MSG(fmt, ...) \
		do { xf86Msg(X_WARNING, fmt "\n",\
				##__VA_ARGS__); } while (0)
*/

//#define VIV2D_INFO_MSG(fmt, ...)
#define VIV2D_INFO_MSG(fmt, ...) \
		do { xf86Msg(X_INFO, fmt "\n",\
				##__VA_ARGS__); } while (0)

//#define VIV2D_ERR_MSG(fmt, ...)
#define VIV2D_ERR_MSG(fmt, ...) \
		do { xf86Msg(X_ERROR, fmt "\n",\
				##__VA_ARGS__); } while (0)

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

typedef struct _Viv2DRect {
	int x1;
	int y1;
	int x2;
	int y2;
} Viv2DRect;

typedef struct _Viv2DFormat {
	int exaFmt;
	int bpp;
	unsigned int fmt;
	int swizzle;
	int alphaBits;
} Viv2DFormat;

typedef struct {
	void *priv;			/* EXA submodule private data */
	struct etna_bo *bo;
	int width;
	int height;
	int pitch;
	Viv2DFormat format;
	Bool tiled;
} Viv2DPixmapPrivRec, *Viv2DPixmapPrivPtr;

typedef struct _Viv2DBlendOp {
	int op;
	int srcBlendMode;
	int dstBlendMode;
	uint8_t src_alpha;
	uint8_t dst_alpha;
} Viv2DBlendOp;

typedef struct _Viv2DOp {
	Viv2DBlendOp *blend_op;
	Viv2DBlendOp *msk_op;
	int repeat;

	uint32_t fg;
	uint32_t mask;

	Viv2DPixmapPrivPtr src;
	Viv2DPixmapPrivPtr msk;
	Viv2DPixmapPrivPtr dst;

	int prev_src_x;
	int prev_src_y;
	int cur_rect;
	Viv2DRect rects[VIV2D_MAX_RECTS];

} Viv2DOp;

typedef struct _Viv2DRec {
	int fd;
	char *render_node;
	struct etna_device *dev;
	struct etna_gpu *gpu;
	struct etna_pipe *pipe;
	struct etna_cmd_stream *stream;

	Viv2DOp *op;

	struct etna_bo *bo;
	int width;
	int height;

} Viv2DRec, *Viv2DPtr;

#ifdef ETNA_BO_FROM_HANDLE_MISSING
// priv etna_bo structure
struct etna_bo {
	struct etna_device      *dev;
	void            *map;           /* userspace mmap'ing (if there is one) */
	uint32_t        size;
	uint32_t        handle;
	uint32_t        flags;
	uint32_t        name;           /* flink global handle (DRI2 name) */
	uint64_t        offset;
	uint32_t        refcnt; // atomic_t

	/* in the common case, a bo won't be referenced by more than a single
	 * command stream.  So to avoid looping over all the bo's in the
	 * reloc table to find the idx of a bo that might already be in the
	 * table, we cache the idx in the bo.  But in order to detect the
	 * slow-path where bo is ref'd in multiple streams, we also must track
	 * the current_stream for which the idx is valid.  See bo2idx().
	 */
	struct etna_cmd_stream *current_stream;
	uint32_t idx;

	int reuse;
	/* more things we don't need */
};


static struct etna_bo *etna_bo_alloc(struct etna_device *dev)
{
	struct etna_bo *mem;

	mem = calloc(1, sizeof *mem);
	if (mem) {
		mem->dev = dev;
		mem->refcnt = 1;
		mem->idx = -1;
	}
	return mem;
}
#endif

#endif