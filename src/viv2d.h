#ifndef VIV2D_H
#define VIV2D_H

#define VIV2D_MAX_RECTS 256


//#define VIV2D_MSG(fmt, ...)
#define VIV2D_MSG(fmt, ...) \
		do { xf86Msg(X_INFO, fmt "\n",\
				##__VA_ARGS__); } while (0)


#define VIV2D_FAIL_MSG(fmt, ...)
/*#define VIV2D_FAIL_MSG(fmt, ...) \
		do { xf86Msg(X_WARNING, fmt "\n",\
				##__VA_ARGS__); } while (0)
*/

//#define VIV2D_ERR_MSG(fmt, ...)
#define VIV2D_ERR_MSG(fmt, ...) \
		do { xf86Msg(X_ERROR, fmt "\n",\
				##__VA_ARGS__); } while (0)

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


typedef struct _Viv2DCompositeOp {
	Viv2DBlendOp blendOp;
/*	Viv2DPictFormat srcFmt;
	Viv2DPictFormat dstFmt;
*/
	Viv2DPixmapPrivPtr src;
	Viv2DPixmapPrivPtr dst;

} Viv2DCompositeOp;

typedef struct _Viv2DRec {
	int fd;
	char *render_node;
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


#endif