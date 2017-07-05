/*	$NetBSD: uonerng.c,v 1.45 2002/09/23 05:51:23 simonb Exp $	*/

/*-
 * Copyright (c) 2003, M. Warner Losh <imp@FreeBSD.org>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net) at
 * Carlstedt Research & Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Comm Class spec:  http://www.usb.org/developers/devclass_docs/usbccs10.pdf
 *                   http://www.usb.org/developers/devclass_docs/usbcdc11.pdf
 *                   http://www.usb.org/developers/devclass_docs/cdc_wmc10.zip
 */

#include <sys/stdint.h>
#include <sys/param.h>
#include <sys/taskqueue.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>
#include <sys/serial.h>
#include <sys/kthread.h>
#include <sys/condvar.h>
#include <sys/random.h>

#include <bus/u4b/usb.h>
#include <bus/u4b/usbdi.h>
#include <bus/u4b/usbdi_util.h>
#include <bus/u4b/usbhid.h>
#include <bus/u4b/usb_cdc.h>
#include <bus/u4b/usb_process.h>
#include "usbdevs.h"
#include "usb_if.h"

#include <bus/u4b/usb_request.h>
#include <bus/u4b/usb_process.h>

#include <bus/u4b/usb_ioctl.h>

#define	USB_DEBUG_VAR uonerng_debug
#include <bus/u4b/usb_debug.h>
#include <bus/u4b/usb_process.h>
#include <bus/u4b/quirk/usb_quirk.h>

#include <bus/u4b/serial/usb_serial.h>

#define	uonerng_MODVER			1	/* module version */

#define ONERNG_BUFSIZ		512
#define ONERNG_MSECS		100
#define ONERNG_TIMEOUT  1000	/* ms */

/* OneRNG operational modes */
#define ONERNG_OP_ENABLE	"cmdO\n" /* start emitting data */
#define ONERNG_OP_DISABLE	"cmdo\n" /* stop emitting data */
#define ONERNG_OP_FLUSH_ENTROPY	"cmdw\n"

/* permits extracting the firmware in order to check the crypto signature */
#define ONERNG_OP_EXTRACT_FIRMWARE "cmdX\n"

/*
 * Noise sources include an avalache circuit and an RF circuit.
 * There is also a whitener to provide a uniform distribution.
 * Different combinations are possible.
 */
#define ONERNG_AVALANCHE_WHITENER	"cmd0\n" /* device default */
#define ONERNG_AVALANCHE		"cmd1\n"
#define ONERNG_AVALANCHE_RF_WHITENER	"cmd2\n"
#define ONERNG_AVALANCHE_RF		"cmd3\n"
#define ONERNG_SILENT			"cmd4\n" /* none; necessary for cmdX */
#define ONERNG_SILENT2			"cmd5\n"
#define ONERNG_RF_WHITENER		"cmd6\n"
#define ONERNG_RF			"cmd7\n"

#define ONERNG_IFACE_CTRL_INDEX	0
#define ONERNG_IFACE_DATA_INDEX	1

#define DEVNAME(_sc) ("OneRNG")

#define	UONERNG_MTX_ASSERT(sc, what) KKASSERT(lockowned((sc)->sc_lock))
#define	UONERNG_MTX_LOCK(sc) lockmgr((sc)->sc_lock, LK_EXCLUSIVE)
#define	UONERNG_MTX_UNLOCK(sc) lockmgr((sc)->sc_lock, LK_RELEASE)

#ifdef USB_DEBUG
static int uonerng_debug = 0;

static SYSCTL_NODE(_hw_usb, OID_AUTO, uonerng, CTLFLAG_RW, 0, "USB uonerng");
SYSCTL_INT(_hw_usb_uonerng, OID_AUTO, debug, CTLFLAG_RW,
    &uonerng_debug, 0, "Debug level");
#endif

static const STRUCT_USB_HOST_ID uonerng_host_devs[] = {
        /* Altus Metrum OneRNG */
        {USB_VPI(USB_VENDOR_OPENMOKO, USB_PRODUCT_OPENMOKO_ONERNG, 1)},
};

#define TTY_NAME "onerng"

typedef struct uonerng_buf {
  char buffer[ONERNG_BUFSIZ];
  int length;
} uonerng_buf_t;

enum {
	uonerng_BULK_WR,
	uonerng_BULK_RD,
	uonerng_INTR_WR,
	uonerng_INTR_RD,
	uonerng_N_TRANSFER,
};

struct uonerng_cfg_task {
	struct usb_proc_msg hdr;
	struct uonerng_softc *sc;
};

struct uonerng_param_task {
	struct usb_proc_msg hdr;
	struct uonerng_softc *sc;
	struct termios termios_copy;
};

struct uonerng_softc {
	struct device *sc_dev;
	struct usb_device *sc_udev;

	struct usb_xfer *sc_xfer[uonerng_N_TRANSFER];
	struct lock sc_lock;

  uonerng_buf_t *tx_buffer;
  uonerng_buf_t *rx_buffer;

  struct usb_process sc_tq;

  struct uonerng_cfg_task	sc_start_task[2];

  struct usb_proc_msg	*sc_last_start_xfer;

	uint8_t	sc_lsr;			/* local status register */
	uint8_t	sc_msr;			/* modem status register */
  uint8_t sc_dtr;
  uint8_t sc_rts;

  uint8_t	sc_ctrl_iface_no;
	uint8_t	sc_data_iface_no;
	uint8_t sc_iface_index[2];
	uint8_t	sc_cm_over_data;
	uint8_t	sc_cm_cap;		/* CM capabilities */
	uint8_t	sc_acm_cap;		/* ACM capabilities */
	uint8_t	sc_line_coding[32];	/* used in USB device mode */
	uint8_t	sc_abstract_state[32];	/* used in USB device mode */

  uint8_t sc_zlps; // zero length packet counter

  int	    sc_ctl_iface_no;			// control
	struct	usbd_interface *sc_data_iface;	// data
};

static device_probe_t uonerng_probe;
static device_attach_t uonerng_attach;
static device_detach_t uonerng_detach;
static usb_handle_request_t uonerng_handle_request;

static usb_callback_t uonerng_intr_read_callback;
static usb_callback_t uonerng_intr_write_callback;
static usb_callback_t uonerng_write_callback;
static usb_callback_t uonerng_read_callback;

static usb_proc_callback_t uonerng_cfg_start_transfers;
static usb_proc_callback_t uonerng_cfg_stop_transfers;

static int uonerng_start(struct uonerng_softc *sc);
static void uonerng_stop(struct uonerng_softc *sc);

static void uonerng_set_line_state(struct uonerng_softc *sc);
static void uonerng_rts(struct uonerng_softc *sc, int onoff);

static void	uonerng_start_read(struct uonerng_softc *);
static void	uonerng_stop_read(struct uonerng_softc *);
static void	uonerng_start_write(struct uonerng_softc *);
static void	uonerng_stop_write(struct uonerng_softc *);
static void	*uonerng_get_desc(struct usb_attach_arg *, uint8_t, uint8_t);
static void	uonerng_find_data_iface(struct usb_attach_arg *uaa,
		    uint8_t, uint8_t *, uint8_t *);
static void uonerng_queue_command(struct uonerng_softc *sc,
            usb_proc_callback_t *fn, struct termios *pt,
            struct usb_proc_msg *t0, struct usb_proc_msg *t1);
void
uonerng_put_data(struct uonerng_softc *, struct usb_page_cache *,
    uint32_t, uint32_t);

static const struct usb_config uonerng_config[uonerng_N_TRANSFER] = {

	[uonerng_BULK_WR] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_TX,
		.if_index = 0,
		.bufsize = ONERNG_BUFSIZ,
		.flags = {.pipe_bof = 1,.force_short_xfer = 1,},
		.callback = &uonerng_write_callback,
		.usb_mode = USB_MODE_DUAL,
	},

	[uonerng_BULK_RD] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_RX,
		.if_index = 0,
		.bufsize = ONERNG_BUFSIZ,
		.flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
		.callback = &uonerng_read_callback,
		.usb_mode = USB_MODE_DUAL,
	},

	[uonerng_INTR_WR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_TX,
		.if_index = 1,
		.flags = {.pipe_bof = 1,.short_xfer_ok = 1,.no_pipe_ok = 1,},
		.bufsize = 0,	/* use wMaxPacketSize */
		.callback = &uonerng_intr_write_callback,
		.usb_mode = USB_MODE_DEVICE,
	},

	[uonerng_INTR_RD] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_RX,
		.if_index = 1,
		.flags = {.pipe_bof = 1,.short_xfer_ok = 1,.no_pipe_ok = 1,},
		.bufsize = 0,	/* use wMaxPacketSize */
		.callback = &uonerng_intr_read_callback,
		.usb_mode = USB_MODE_HOST,
	},
};

static device_method_t uonerng_methods[] = {
	/* USB interface */
	DEVMETHOD(usb_handle_request, uonerng_handle_request),

	/* Device interface */
	DEVMETHOD(device_probe, uonerng_probe),
	DEVMETHOD(device_attach, uonerng_attach),
	DEVMETHOD(device_detach, uonerng_detach),
	DEVMETHOD_END
};

static devclass_t uonerng_devclass;

static driver_t uonerng_driver = {
	.name = "uonerng",
	.methods = uonerng_methods,
	.size = sizeof(struct uonerng_softc),
};

DRIVER_MODULE(uonerng, uhub, uonerng_driver, uonerng_devclass, NULL, NULL);
MODULE_DEPEND(uonerng, usb, 1, 1, 1);
MODULE_VERSION(uonerng, uonerng_MODVER);

MALLOC_DECLARE(UONERNG_BUF);
MALLOC_DEFINE(UONERNG_BUF, "uonerng_buf", "buffer for uonerng");

static int uonerng_probe(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	int error;

	DPRINTFN(11, "\n");

	error = usbd_lookup_id_by_uaa(uonerng_host_devs,
	    sizeof(uonerng_host_devs), uaa);
	if (error) {
			return (error);
	}
	return (BUS_PROBE_GENERIC);
}

static int
uonerng_attach(device_t dev)
{
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	struct uonerng_softc *sc = device_get_softc(dev);
	struct usb_cdc_cm_descriptor *cmd;
	struct usb_cdc_union_descriptor *cud;
	uint8_t i;
	int error;

	device_set_usb_desc(dev);
	lockinit(&sc->sc_lock, "uonerng", 0, LK_CANRECURSE);

	sc->sc_ctrl_iface_no = uaa->info.bIfaceNum;
	sc->sc_iface_index[1] = uaa->info.bIfaceIndex;
	sc->sc_udev = uaa->device;

  sc->sc_dtr = -1;
  sc->sc_rts = -1;

  sc->tx_buffer = kmalloc(sizeof(uonerng_buf_t), UONERNG_BUF, M_WAITOK);
  if(!sc->tx_buffer) {
    DPRINTF("Failed to allocate tx_buffer!\n");
  }
  sc->rx_buffer = kmalloc(sizeof(uonerng_buf_t), UONERNG_BUF, M_WAITOK);
  if(!sc->rx_buffer) {
    DPRINTF("Failed to allocate rx_buffer");
  }


	/* get the data interface number */
	cmd = uonerng_get_desc(uaa, UDESC_CS_INTERFACE, UDESCSUB_CDC_CM);

	if ((cmd == NULL) || (cmd->bLength < sizeof(*cmd))) {

		cud = usbd_find_descriptor(uaa->device, NULL,
		    uaa->info.bIfaceIndex, UDESC_CS_INTERFACE,
		    0 - 1, UDESCSUB_CDC_UNION, 0 - 1);

		if ((cud == NULL) || (cud->bLength < sizeof(*cud))) {
			DPRINTF("Missing descriptor. "
			    "Assuming data interface is next.\n");
			if (sc->sc_ctrl_iface_no == 0xFF) {
				goto detach;
			} else {
				uint8_t class_match = 0;

				/* set default interface number */
				sc->sc_data_iface_no = 0xFF;

				/* try to find the data interface backwards */
				uonerng_find_data_iface(uaa,
				    uaa->info.bIfaceIndex - 1,
				    &sc->sc_data_iface_no, &class_match);

				/* try to find the data interface forwards */
				uonerng_find_data_iface(uaa,
				    uaa->info.bIfaceIndex + 1,
				    &sc->sc_data_iface_no, &class_match);

				/* check if nothing was found */
				if (sc->sc_data_iface_no == 0xFF)
					goto detach;
			}
		} else {
			sc->sc_data_iface_no = cud->bSlaveInterface[0];
		}
	} else {
		sc->sc_data_iface_no = cmd->bDataInterface;
	}

	/* get the data interface too */
	for (i = 0;; i++) {
		struct usb_interface *iface;
		struct usb_interface_descriptor *id;

		iface = usbd_get_iface(uaa->device, i);

		if (iface) {

			id = usbd_get_interface_descriptor(iface);

			if (id && (id->bInterfaceNumber == sc->sc_data_iface_no)) {
				sc->sc_iface_index[0] = i;
				usbd_set_parent_iface(uaa->device, i, uaa->info.bIfaceIndex);
				break;
			}
		} else {
			device_printf(dev, "no data interface\n");
			goto detach;
		}
	}

	error = usbd_transfer_setup(uaa->device,
	    sc->sc_iface_index, sc->sc_xfer,
	    uonerng_config, uonerng_N_TRANSFER,
	    sc, &sc->sc_lock);
	if (error) {
		device_printf(dev, "Can't setup transfer\n");
		goto detach;
	}

	/* clear stall at first run, if USB host mode */
	if (uaa->usb_mode == USB_MODE_HOST) {
		lockmgr(&sc->sc_lock, LK_EXCLUSIVE);
		usbd_xfer_set_stall(sc->sc_xfer[uonerng_BULK_WR]);
		usbd_xfer_set_stall(sc->sc_xfer[uonerng_BULK_RD]);
		lockmgr(&sc->sc_lock, LK_RELEASE);
	}

  uonerng_start(sc);

	return (0);

detach:
	uonerng_detach(dev);
	return (ENXIO);
}

static void
uonerng_find_data_iface(struct usb_attach_arg *uaa,
    uint8_t iface_index, uint8_t *p_data_no, uint8_t *p_match_class)
{
	struct usb_interface_descriptor *id;
	struct usb_interface *iface;

	iface = usbd_get_iface(uaa->device, iface_index);

	/* check for end of interfaces */
	if (iface == NULL)
		return;

	id = usbd_get_interface_descriptor(iface);

	/* check for non-matching interface class */
	if (id->bInterfaceClass != UICLASS_CDC_DATA ||
	    id->bInterfaceSubClass != UISUBCLASS_DATA) {
		/* if we got a class match then return */
		if (*p_match_class)
			return;
	} else {
		*p_match_class = 1;
	}

	DPRINTFN(11, "Match at index %u\n", iface_index);

	*p_data_no = id->bInterfaceNumber;
}

static void
uonerng_start_read(struct uonerng_softc *sc)
{
	/* start interrupt endpoint, if any */
	usbd_transfer_start(sc->sc_xfer[uonerng_INTR_RD]);

	/* start read endpoint */
	usbd_transfer_start(sc->sc_xfer[uonerng_BULK_RD]);
}

static void
uonerng_stop_read(struct uonerng_softc *sc)
{
	/* stop interrupt endpoint, if any */
	usbd_transfer_stop(sc->sc_xfer[uonerng_INTR_RD]);

	/* stop read endpoint */
	usbd_transfer_stop(sc->sc_xfer[uonerng_BULK_RD]);
}

static void
uonerng_start_write(struct uonerng_softc *sc)
{
	usbd_transfer_start(sc->sc_xfer[uonerng_INTR_WR]);
	usbd_transfer_start(sc->sc_xfer[uonerng_BULK_WR]);
}

static void
uonerng_stop_write(struct uonerng_softc *sc)
{
	usbd_transfer_stop(sc->sc_xfer[uonerng_INTR_WR]);
	usbd_transfer_stop(sc->sc_xfer[uonerng_BULK_WR]);
}

static void
uonerng_intr_write_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int actlen;
	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:

		DPRINTF("Transferred %d bytes\n", actlen);

		/* FALLTHROUGH */
	case USB_ST_SETUP:
tr_setup:
		break;

	default:			/* Error */
		if (error != USB_ERR_CANCELLED) {
			/* start clear stall */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
}

static void
uonerng_intr_read_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct usb_cdc_notification pkt;
	struct uonerng_softc *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	uint16_t wLen;
	int actlen;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:

		if (actlen < 8) {
			DPRINTF("received short packet, "
			    "%d bytes\n", actlen);
			goto tr_setup;
		}
		if (actlen > sizeof(pkt)) {
			DPRINTF("truncating message\n");
			actlen = sizeof(pkt);
		}
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, &pkt, actlen);

		actlen -= 8;

		wLen = UGETW(pkt.wLength);
		if (actlen > wLen) {
			actlen = wLen;
		}
		if (pkt.bmRequestType != UCDC_NOTIFICATION) {
			DPRINTF("unknown message type, "
			    "0x%02x, on notify pipe!\n",
			    pkt.bmRequestType);
			goto tr_setup;
		}
		switch (pkt.bNotification) {
		case UCDC_N_SERIAL_STATE:
			/*
			 * Set the serial state in ucom driver based on
			 * the bits from the notify message
			 */
			if (actlen < 2) {
				DPRINTF("invalid notification "
				    "length, %d bytes!\n", actlen);
				break;
			}
			DPRINTF("notify bytes = %02x%02x\n",
			    pkt.data[0],
			    pkt.data[1]);

			/* Currently, lsr is always zero. */
			sc->sc_lsr = 0;
			sc->sc_msr = 0;

			if (pkt.data[0] & UCDC_N_SERIAL_RI) {
				sc->sc_msr |= SER_RI;
			}
			if (pkt.data[0] & UCDC_N_SERIAL_DSR) {
				sc->sc_msr |= SER_DSR;
			}
			if (pkt.data[0] & UCDC_N_SERIAL_DCD) {
				sc->sc_msr |= SER_DCD;
			}
			//ucom_status_change(&sc->sc_udev);
			break;

		default:
			DPRINTF("unknown notify message: 0x%02x\n",
			    pkt.bNotification);
			break;
		}

	case USB_ST_SETUP:
tr_setup:
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		return;

	default:			/* Error */
		if (error != USB_ERR_CANCELLED) {
			/* try to clear stall first */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		return;

	}
}

static void
uonerng_write_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct uonerng_softc *sc = usbd_xfer_softc(xfer);
  struct usb_page_cache *pc;
  uonerng_buf_t *buf = sc->tx_buffer;

	switch (USB_GET_STATE(xfer)) {

	case USB_ST_TRANSFERRED:
    buf->length = 0;
  case USB_ST_SETUP:
tr_setup:
		pc = usbd_xfer_get_frame(xfer, 0);
    if(buf->length > 0) {
      usbd_copy_in(pc, 0, buf->buffer, buf->length);
     	usbd_xfer_set_frame_len(xfer, 0, buf->length);
  		usbd_transfer_submit(xfer);
  	}

    break;

	default:			/* Error */
		if (error != USB_ERR_CANCELLED) {
			/* try to clear stall first */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		return;
	}
}

static void
uonerng_read_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct uonerng_softc *sc = usbd_xfer_softc(xfer);
  uonerng_buf_t *buf = sc->tx_buffer;
	struct usb_page_cache *pc;
	int actual, max;

	usbd_xfer_status(xfer, &actual, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
    if(actual == 0) {
      if(sc->sc_zlps == 4) {
        /* Throttle transfers */
        usbd_xfer_set_interval(xfer, 500);
      } else {
        sc->sc_zlps++;
      }
    } else {
      /* Disable throttling */
      usbd_xfer_set_interval(xfer, 0);
      sc->sc_zlps = 0;
    }

		DPRINTF("actlen=%d\n", actual);
		pc = usbd_xfer_get_frame(xfer, 0);
    uonerng_put_data(sc, pc, 0, actual);
    buf->length = actual;

	case USB_ST_SETUP:
tr_setup:
		max = usbd_xfer_max_len(xfer);
    usbd_xfer_set_frame_len(xfer, 0, max);
		usbd_transfer_submit(xfer);
		return;

	default:
    /* Disable throttling */
    usbd_xfer_set_interval(xfer, 0);
    sc->sc_zlps = 0;

    /* Error */
		if (error != USB_ERR_CANCELLED) {
			/* try to clear stall first */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		return;
	}
}

static void *
uonerng_get_desc(struct usb_attach_arg *uaa, uint8_t type, uint8_t subtype)
{
	return (usbd_find_descriptor(uaa->device, NULL, uaa->info.bIfaceIndex,
	    type, 0 - 1, subtype, 0 - 1));
}


static int
uonerng_detach(device_t dev)
{
	struct uonerng_softc *sc = device_get_softc(dev);

  uonerng_stop_write(sc);
  uonerng_stop_read(sc);

	DPRINTF("sc=%p\n", sc);
  uonerng_stop(sc);
  usb_proc_drain(&sc->sc_tq);
  usb_proc_free(&sc->sc_tq);

  kfree(sc->tx_buffer, UONERNG_BUF);
  kfree(sc->rx_buffer, UONERNG_BUF);

	usbd_transfer_unsetup(sc->sc_xfer, uonerng_N_TRANSFER);
	lockuninit(&sc->sc_lock);

	return (0);
}

static int
uonerng_handle_request(device_t dev,
    const void *preq, void **pptr, uint16_t *plen,
    uint16_t offset, uint8_t *pstate)
{
	struct uonerng_softc *sc = device_get_softc(dev);
	const struct usb_device_request *req = preq;
	uint8_t is_complete = *pstate;
	DPRINTF("sc=%p\n", sc);

	if (!is_complete) {
		if ((req->bmRequestType == UT_WRITE_CLASS_INTERFACE) &&
		    (req->bRequest == UCDC_SET_LINE_CODING) &&
		    (req->wIndex[0] == sc->sc_ctrl_iface_no) &&
		    (req->wIndex[1] == 0x00) &&
		    (req->wValue[0] == 0x00) &&
		    (req->wValue[1] == 0x00)) {
			if (offset == 0) {
				*plen = sizeof(sc->sc_line_coding);
				*pptr = &sc->sc_line_coding;
			} else {
				*plen = 0;
			}
			return (0);
		} else if ((req->bmRequestType == UT_WRITE_CLASS_INTERFACE) &&
		    (req->wIndex[0] == sc->sc_ctrl_iface_no) &&
		    (req->wIndex[1] == 0x00) &&
		    (req->bRequest == UCDC_SET_COMM_FEATURE)) {
			if (offset == 0) {
				*plen = sizeof(sc->sc_abstract_state);
				*pptr = &sc->sc_abstract_state;
			} else {
				*plen = 0;
			}
			return (0);
		} else if ((req->bmRequestType == UT_WRITE_CLASS_INTERFACE) &&
		    (req->wIndex[0] == sc->sc_ctrl_iface_no) &&
		    (req->wIndex[1] == 0x00) &&
		    (req->bRequest == UCDC_SET_CONTROL_LINE_STATE)) {
			*plen = 0;
			return (0);
		} else if ((req->bmRequestType == UT_WRITE_CLASS_INTERFACE) &&
		    (req->wIndex[0] == sc->sc_ctrl_iface_no) &&
		    (req->wIndex[1] == 0x00) &&
		    (req->bRequest == UCDC_SEND_BREAK)) {
			*plen = 0;
			return (0);
		}
	}
	return (ENXIO);			/* use builtin handler */
}

/* Send a command to the OneRNG */
static void uonerng_send_cmd(struct usb_proc_msg *_task)
{
  struct uonerng_cfg_task *task =
      (struct uonerng_cfg_task *)_task;
  struct uonerng_softc *sc = task->sc;

  uonerng_buf_t *buffer = sc->tx_buffer;
  buffer->length = sizeof(ONERNG_OP_ENABLE);
  strcpy(buffer->buffer, ONERNG_OP_ENABLE);
  //ksnprintf(buffer->buffer, buffer->length, "%s",ONERNG_OP_ENABLE);
}

static int uonerng_start(struct uonerng_softc *sc)
{
  int error = 0;

  DPRINTF("Starting OneRNG!\n");

  /* create USB request handling process */
  DPRINTF("Creating usb process...\n");
  error = usb_proc_create(&sc->sc_tq, &sc->sc_lock, "uonerng", USB_PRI_MED);
  if (error) {
    return (error);
  }
  lockmgr(&sc->sc_lock, LK_EXCLUSIVE);

  // Clear RTS
  uonerng_rts(sc, 0);

  uonerng_queue_command(sc, uonerng_send_cmd, NULL,
    &sc->sc_start_task[0].hdr,
    &sc->sc_start_task[1].hdr);

  /* Queue transfer enable command last */
  uonerng_queue_command(sc, uonerng_cfg_start_transfers, NULL,
      &sc->sc_start_task[0].hdr,
      &sc->sc_start_task[1].hdr);

  lockmgr(&sc->sc_lock, LK_RELEASE);

  return 0;
}

static void uonerng_stop(struct uonerng_softc *sc)
{
  DPRINTF("Stopping OneRNG!\n");

  uonerng_queue_command(sc, uonerng_cfg_stop_transfers, NULL,
      &sc->sc_start_task[0].hdr,
      &sc->sc_start_task[1].hdr);
  //uonerng_send_cmd(sc, ONERNG_OP_DISABLE);

  return;
}

static void uonerng_rts(struct uonerng_softc *sc, int onoff)
{
	if (sc->sc_rts == onoff)
		return;
	sc->sc_rts = onoff;

	uonerng_set_line_state(sc);
}

void uonerng_set_line_state(struct uonerng_softc *sc)
{
	usb_device_request_t req;
	int ls;

	ls = (sc->sc_dtr ? UCDC_LINE_DTR : 0) |
	     (sc->sc_rts ? UCDC_LINE_RTS : 0);
	req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
	req.bRequest = UCDC_SET_CONTROL_LINE_STATE;
	USETW(req.wValue, ls);
	USETW(req.wIndex, sc->sc_ctl_iface_no);
	USETW(req.wLength, 0);

  usbd_do_request(sc->sc_udev, NULL, &req, 0);
}

static void
uonerng_cfg_start_transfers(struct usb_proc_msg *_task)
{
	struct uonerng_cfg_task *task =
	    (struct uonerng_cfg_task *)_task;
	struct uonerng_softc *sc = task->sc;

	uonerng_start_read(sc);
	uonerng_start_write(sc);
}

static void
uonerng_cfg_stop_transfers(struct usb_proc_msg *_task)
{
	struct uonerng_cfg_task *task =
	    (struct uonerng_cfg_task *)_task;
	struct uonerng_softc *sc = task->sc;

	uonerng_stop_read(sc);
	uonerng_stop_write(sc);
}

static void
uonerng_queue_command(struct uonerng_softc *sc,
    usb_proc_callback_t *fn, struct termios *pt,
    struct usb_proc_msg *t0, struct usb_proc_msg *t1)
{
	struct uonerng_param_task *task;

	UONERNG_MTX_ASSERT(&sc->sc_tq, MA_OWNED);

	if (usb_proc_is_gone(&sc->sc_tq)) {
		DPRINTF("proc is gone\n");
		return;         /* nothing to do */
	}
	/*
	 * NOTE: The task cannot get executed before we drop the
	 * "sc_lock" lock. It is safe to update fields in the message
	 * structure after that the message got queued.
	 */
	task = (struct uonerng_param_task *)
	  usb_proc_msignal(&sc->sc_tq, t0, t1);

	/* Setup callback and softc pointers */
	task->hdr.pm_callback = fn;
	task->sc = sc;

	/*
	 * Make a copy of the termios. This field is only present if
	 * the "pt" field is not NULL.
	 */
	if (pt != NULL)
		task->termios_copy = *pt;
  //
	// /*
	//  * Closing the device should be synchronous.
	//  */
	// if (fn == uonerng_cfg_close)
	// 	usb_proc_mwait(&sc->sc_tq, t0, t1);

	/*
	 * In case of multiple configure requests,
	 * keep track of the last one!
	 */
	if (fn == uonerng_cfg_start_transfers)
		sc->sc_last_start_xfer = &task->hdr;
}

void
uonerng_put_data(struct uonerng_softc *sc, struct usb_page_cache *pc,
    uint32_t offset, uint32_t len)
{
	char *buf;

	DPRINTF("\n");

	UONERNG_MTX_ASSERT(&sc->sc_tq, MA_OWNED);

  buf = kmalloc(len, UONERNG_BUF, M_WAITOK);
  if(!buf) {
    DPRINTF("Failed to allocate tx_buffer!\n");
    return;
  }

  /* Copy out the data */
  usbd_copy_out(pc, offset, buf, len);

  /* Add randomness to system */
  add_buffer_randomness(buf, len);

  /* Free buffer */
  kfree(buf, UONERNG_BUF);

	return;
}
