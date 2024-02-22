/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <controller/ble_ll.h>
#include <controller/ble_ll_iso.h>
#include <controller/ble_ll_isoal.h>
#include <controller/ble_ll_tmr.h>
#include <controller/ble_ll_utils.h>
#include <stdint.h>
#include <syscfg/syscfg.h>

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#if MYNEWT_VAL(BLE_LL_ISO)

void
ble_ll_isoal_mux_init(struct ble_ll_isoal_mux *mux,
                      const struct ble_ll_isoal_mux_init_param *param)
{
    memset(mux, 0, sizeof(*mux));

    mux->iso_interval_us = param->iso_interval_us;
    mux->sdu_interval_us = param->sdu_interval_us;

    mux->max_sdu = param->max_sdu;
    mux->max_pdu = param->max_pdu;
    /* Core 5.3, Vol 6, Part G, 2.1 */
    mux->sdu_per_interval = param->iso_interval_us / param->sdu_interval_us;

    if (param->framed) {
        /* TODO */
    } else {
        mux->pdu_per_sdu = param->bn / mux->sdu_per_interval;
    }

    mux->sdu_per_event = (1 + param->pte) * mux->sdu_per_interval;

    mux->bn = param->bn;

    STAILQ_INIT(&mux->sdu_q);
    mux->sdu_q_len = 0;

    STAILQ_INIT(&mux->pdu_q);

    mux->framed = param->framed;
    mux->framing_mode = param->framing_mode;
}

void
ble_ll_isoal_mux_free(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *pkthdr;
    struct os_mbuf *om;
    struct os_mbuf *om_next;

    pkthdr = STAILQ_FIRST(&mux->sdu_q);
    while (pkthdr) {
        om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);

        while (om) {
            om_next = SLIST_NEXT(om, om_next);
            os_mbuf_free(om);
            om = om_next;
        }

        STAILQ_REMOVE_HEAD(&mux->sdu_q, omp_next);
        pkthdr = STAILQ_FIRST(&mux->sdu_q);
    }

    STAILQ_INIT(&mux->sdu_q);
}

void
ble_ll_isoal_mux_sdu_enqueue(struct ble_ll_isoal_mux *mux, struct os_mbuf *om)
{
    struct os_mbuf_pkthdr *pkthdr;
    os_sr_t sr;

    BLE_LL_ASSERT(mux);

    OS_ENTER_CRITICAL(sr);
    pkthdr = OS_MBUF_PKTHDR(om);
    STAILQ_INSERT_TAIL(&mux->sdu_q, pkthdr, omp_next);
    mux->sdu_q_len++;
#if MYNEWT_VAL(BLE_LL_ISOAL_MUX_PREFILL)
    if (mux->sdu_q_len >= mux->sdu_per_event) {
        mux->active = 1;
    }
#endif
    OS_EXIT_CRITICAL(sr);
}

int
ble_ll_isoal_mux_event_start(struct ble_ll_isoal_mux *mux, uint32_t timestamp)
{
#if MYNEWT_VAL(BLE_LL_ISOAL_MUX_PREFILL)
    /* If prefill is enabled, we always expect to have required number of SDUs
     * in queue, otherwise we disable mux until enough SDUs are queued again.
     */
    if (mux->sdu_per_event > mux->sdu_q_len) {
        mux->active = 0;
    }
    if (mux->active && mux->framed) {
        mux->sdu_in_event = mux->sdu_q_len;
    } else if (mux->active) {
        mux->sdu_in_event = mux->sdu_per_event;
    } else {
        mux->sdu_in_event = 0;
    }
#else
    if (mux->framed) {
        mux->sdu_in_event = mux->sdu_q_len;
    } else {
        mux->sdu_in_event = min(mux->sdu_q_len, mux->sdu_per_event);
    }
#endif

    mux->event_tx_timestamp = timestamp;

    return mux->sdu_in_event;
}

static int
ble_ll_isoal_mux_unframed_event_done(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *pkthdr;
    struct os_mbuf *om;
    struct os_mbuf *om_next;
    uint8_t num_sdu;
    int pkt_freed = 0;
    os_sr_t sr;

    num_sdu = min(mux->sdu_in_event, mux->sdu_per_interval);

#if MYNEWT_VAL(BLE_LL_ISO_HCI_DISCARD_THRESHOLD)
    /* Drop queued SDUs if number of queued SDUs exceeds defined threshold.
     * Threshold is defined as number of ISO events. If number of queued SDUs
     * exceeds number of SDUs required for single event (i.e. including pt)
     * and number of subsequent ISO events defined by threshold value, we'll
     * drop any excessive SDUs and notify host as if they were sent.
     */
    uint32_t thr = MYNEWT_VAL(BLE_LL_ISO_HCI_DISCARD_THRESHOLD);
    if (mux->sdu_q_len > mux->sdu_per_event + thr * mux->sdu_per_interval) {
        num_sdu = mux->sdu_q_len - mux->sdu_per_event -
                  thr * mux->sdu_per_interval;
    }
#endif

    pkthdr = STAILQ_FIRST(&mux->sdu_q);
    while (pkthdr && num_sdu--) {
        OS_ENTER_CRITICAL(sr);
        STAILQ_REMOVE_HEAD(&mux->sdu_q, omp_next);
        BLE_LL_ASSERT(mux->sdu_q_len > 0);
        mux->sdu_q_len--;
        OS_EXIT_CRITICAL(sr);

        om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);
        while (om) {
            om_next = SLIST_NEXT(om, om_next);
            os_mbuf_free(om);
            pkt_freed++;
            om = om_next;
        }

        pkthdr = STAILQ_FIRST(&mux->sdu_q);
    }

    mux->sdu_in_event = 0;

    return pkt_freed;
}

static int
ble_ll_isoal_mux_framed_event_done(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *pkthdr;
    struct os_mbuf *om;
    struct os_mbuf *om_next;
    uint8_t num_sdu;
    uint8_t num_pdu;
    uint8_t pdu_offset = 0;
    uint8_t frag_len = 0;
    uint8_t rem_len = 0;
    uint8_t hdr_len = 0;
    int pkt_freed = 0;
    bool sc = mux->sc;
    os_sr_t sr;

    num_sdu = mux->sdu_in_event;
    if (num_sdu == 0) {
        return 0;
    }

    num_pdu = mux->bn;

#if MYNEWT_VAL(BLE_LL_ISO_HCI_DISCARD_THRESHOLD)
    /* Drop queued SDUs if number of queued SDUs exceeds defined threshold.
     * Threshold is defined as number of ISO events. If number of queued SDUs
     * exceeds number of SDUs required for single event (i.e. including pt)
     * and number of subsequent ISO events defined by threshold value, we'll
     * drop any excessive SDUs and notify host as if they were sent.
     */
    uint32_t thr = MYNEWT_VAL(BLE_LL_ISO_HCI_DISCARD_THRESHOLD);
    if (mux->sdu_q_len > mux->sdu_per_event + thr * mux->sdu_per_interval) {
        num_sdu = mux->sdu_q_len - mux->sdu_per_event -
                  thr * mux->sdu_per_interval;
    }
#endif

    /* Drop num_pdu PDUs */
    pkthdr = STAILQ_FIRST(&mux->sdu_q);
    while (pkthdr && num_sdu--) {
        om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);

        while (om && num_pdu > 0) {
            rem_len = om->om_len;
            hdr_len = sc ? 2 /* Segmentation Header */
                         : 5 /* Segmentation Header + TimeOffset */;

            if (mux->max_pdu <= hdr_len + pdu_offset) {
                /* Advance to next PDU */
                pdu_offset = 0;
                num_pdu--;
                continue;
            }

            frag_len = min(rem_len, mux->max_pdu - hdr_len - pdu_offset);

            pdu_offset += hdr_len + frag_len;

            os_mbuf_adj(om, frag_len);

            if (frag_len == rem_len) {
                om_next = SLIST_NEXT(om, om_next);
                os_mbuf_free(om);
                pkt_freed++;
                om = om_next;
            } else {
                sc = 1;
            }
        }

        if (num_pdu == 0) {
            break;
        }

        OS_ENTER_CRITICAL(sr);
        STAILQ_REMOVE_HEAD(&mux->sdu_q, omp_next);
        BLE_LL_ASSERT(mux->sdu_q_len > 0);
        mux->sdu_q_len--;
        OS_EXIT_CRITICAL(sr);

        sc = 0;
        pkthdr = STAILQ_FIRST(&mux->sdu_q);
    }

    mux->sdu_in_event = 0;
    mux->sc = sc;

    return pkt_freed;
}

static void
ble_ll_isoal_mux_sdu_emit(struct ble_ll_isoal_mux *mux, struct os_mbuf *sdu,
                          uint32_t time_offset, bool error)
{
    /* Core 6.0 | Vol 6, Part G, 4
     * SDUs with a length exceeding Max_SDU. In this case, the length of the SDU reported
     * to the upper layer shall not exceed the Max_SDU length. The SDU shall be truncated
     * to Max_SDU octets.
     */
    if (os_mbuf_len(sdu) > mux->max_sdu) {
        os_mbuf_adj(sdu, -(os_mbuf_len(sdu) - mux->max_sdu));
        error = true;
    }

    if (mux->cb) {
        mux->cb->sdu_send(mux, sdu, mux->event_tx_timestamp + time_offset,
                          ++mux->sdu_counter, !error);
    }

    os_mbuf_free_chain(sdu);
}

static uint8_t
pdu_idx_get(struct ble_mbuf_hdr *hdr)
{
    return POINTER_TO_UINT(hdr->rxinfo.user_data);
}

static void
pdu_idx_set(struct ble_mbuf_hdr *hdr, uint8_t pdu_idx)
{
    hdr->rxinfo.user_data = UINT_TO_POINTER(pdu_idx);
}

static void
ble_ll_isoal_mux_reassemble(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *entry;
    struct ble_mbuf_hdr *hdr;
    struct os_mbuf *om;
    struct os_mbuf *seg;
    struct os_mbuf *sdu;
    uint32_t time_offset;
    uint16_t seghdr;
    uint8_t hdr_byte;
    uint8_t llid;
    uint8_t len;
    bool start;
    bool cmplt;
    bool error;

    sdu = mux->frag;
    error = false;
    cmplt = false;
    time_offset = 0;

    entry = STAILQ_FIRST(&mux->pdu_q);
    for (uint8_t pdu_idx = 0; pdu_idx < mux->bn; pdu_idx++) {
        if (entry == NULL) {
            error = true;
            /* FIXME */
            break;
        }

        om = OS_MBUF_PKTHDR_TO_MBUF(entry);
        hdr = BLE_MBUF_HDR_PTR(om);

        if (pdu_idx_get(hdr) != pdu_idx) {
            error = true;
            continue;
        }

        /* Remove the PDU from queue */
        STAILQ_REMOVE_HEAD(&mux->pdu_q, omp_next);

        hdr_byte = om->om_data[0];
        BLE_LL_ASSERT(BLE_LL_BIS_LLID_IS_DATA(hdr_byte));

        /* Strip the header from the buffer to process only the payload data */
        os_mbuf_adj(om, BLE_LL_PDU_HDR_LEN);

        llid = hdr_byte & BLE_LL_BIS_PDU_HDR_LLID_MASK;

        if (llid == BLE_LL_BIS_LLID_DATA_PDU_FRAMED) {
            while (om != NULL && os_mbuf_len(om) > 0) {
                om = os_mbuf_pullup(om, sizeof(seghdr));
                if (om == NULL) {
                    error = true;
                    break;
                }

                seghdr = get_le16(om->om_data);
                start = !BLE_LL_ISOAL_SEGHDR_SC(seghdr);
                cmplt = BLE_LL_ISOAL_SEGHDR_CMPLT(seghdr);
                len = BLE_LL_ISOAL_SEGHDR_LEN(seghdr);
                os_mbuf_adj(om, sizeof(seghdr));

                if (start) {
                    /* SDU start */
                    om = os_mbuf_pullup(om, 3);
                    if (om == NULL) {
                        error = true;
                        break;
                    }

                    time_offset = get_le24(om->om_data);
                    os_mbuf_adj(om, 3);
                    len -= 3;

                    if (sdu) {
                        /* Emit the pending SDU with errors */
                        ble_ll_isoal_mux_sdu_emit(mux, sdu, time_offset, true);
                        sdu = NULL;
                    }
                } else if (sdu == NULL) {
                    /* SDU continuation. Drop the segment, since we do not have a start segment */
                    os_mbuf_adj(om, len);
                    error = true;
                    continue;
                }

                seg = om;

                if (len == os_mbuf_len(om)) {
                    /* No more segments in the 'om' */
                    om = NULL;
                } else if (os_mbuf_len(seg) > len) {
                    /* Duplicate and adjust buffer */
                    om = os_mbuf_dup(seg);

                    /* Trim the 'om' head so that 'om->om_data' will point to the data behind SDU Segment */
                    os_mbuf_adj(om, len);
                    len = os_mbuf_len(om);

                    /* Trim the 'frag' tail so that the 'frag' will contain SDU Segment only */
                    os_mbuf_adj(seg, -len);
                } else {
                    /* No more segments in the 'om' */
                    om = NULL;

                    /* Segment length exceeds the length of the buffer */
                    error = true;
                }

                if (start) {
                    BLE_LL_ASSERT(sdu == NULL);
                    sdu = seg;
                } else {
                    BLE_LL_ASSERT(sdu != NULL);
                    sdu = os_mbuf_pack_chains(sdu, seg);
                }

                if (cmplt) {
                    ble_ll_isoal_mux_sdu_emit(mux, sdu, time_offset, error);
                    sdu = NULL;
                    cmplt = false;
                }
            }
        } else {
            /* Invalid LLID, free and move on */
            error = true;
        }

        if (om != NULL) {
            os_mbuf_free_chain(om);
        }

        entry = STAILQ_FIRST(&mux->pdu_q);
    }

    if (sdu != NULL && error) {
        /* Emit the pending SDU with errors */
        ble_ll_isoal_mux_sdu_emit(mux, sdu, time_offset, true);
        sdu = NULL;
    }

    /* Save partially processed SDU */
    mux->frag = sdu;
}

static void
ble_ll_isoal_mux_recombine(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *entry;
    struct ble_mbuf_hdr *hdr;
    struct os_mbuf *om;
    struct os_mbuf *sdu;
    uint8_t hdr_byte;
    uint8_t llid;
    bool cmplt;
    bool error;

    entry = STAILQ_FIRST(&mux->pdu_q);
    for (uint8_t bn = 0; bn < mux->bn; bn += mux->pdu_per_sdu) {
        sdu = NULL;
        error = false;
        cmplt = false;

        for (uint8_t pdu_idx = bn; pdu_idx < bn + mux->pdu_per_sdu; pdu_idx++) {
            if (entry == NULL) {
                error = true;
                break;
            }

            om = OS_MBUF_PKTHDR_TO_MBUF(entry);
            hdr = BLE_MBUF_HDR_PTR(om);

            if (pdu_idx_get(hdr) != pdu_idx) {
                error = true;
                continue;
            }

            /* Remove the PDU from queue */
            STAILQ_REMOVE_HEAD(&mux->pdu_q, omp_next);

            hdr_byte = om->om_data[0];
            BLE_LL_ASSERT(BLE_LL_BIS_LLID_IS_DATA(hdr_byte));

            /* Strip the header from the buffer to process only the payload data */
            os_mbuf_adj(om, BLE_LL_PDU_HDR_LEN);

            llid = hdr_byte & BLE_LL_BIS_PDU_HDR_LLID_MASK;

            if (llid == BLE_LL_BIS_LLID_DATA_PDU_UNFRAMED_CMPLT) {
                /* Unframed BIS Data PDU; end fragment of an SDU or a complete SDU. */
                if (cmplt) {
                    /* Core 6.0 | Vol 6, Part G | 4
                     * Unframed SDUs without exactly one fragment with LLID=0b00 shall be
                     * discarded or reported as data with errors.
                     */
                    error = true;
                }

                if (sdu != NULL) {
                    os_mbuf_concat(sdu, om);
                } else {
                    sdu = om;
                }
                cmplt = true;
            } else if (llid == BLE_LL_BIS_LLID_DATA_PDU_UNFRAMED_SC) {
                /* Unframed BIS Data PDU; start or continuation fragment of an SDU. */
                if (cmplt && om->om_len > BLE_LL_PDU_HDR_LEN) {
                    /* Core 6.0 | Vol 6, Part G | 4
                     * Unframed SDUs where the fragment with LLID=0b00 is followed by a fragment
                     * with LLID=0b01 and containing at least one octet of data shall be discarded
                     * or reported as data with errors.
                     */
                    error = true;
                }

                if (sdu != NULL) {
                    os_mbuf_concat(sdu, om);
                } else {
                    sdu = om;
                }
            } else {
                os_mbuf_free_chain(om);
                error = true;
            }

            entry = STAILQ_FIRST(&mux->pdu_q);
        }

        /* SDU shall be complete */
        error |= !cmplt;

        /* Core 6.0 | Vol 6, Part G, 4
         * SDUs with a length exceeding Max_SDU. In this case, the length of the SDU reported
         * to the upper layer shall not exceed the Max_SDU length. The SDU shall be truncated
         * to Max_SDU octets.
         */
        if (sdu != NULL && os_mbuf_len(sdu) > mux->max_sdu) {
            os_mbuf_adj(sdu, -(os_mbuf_len(sdu) - mux->max_sdu));
            error = true;
        }

        if (mux->cb != NULL) {
            mux->cb->sdu_send(mux, sdu, mux->event_tx_timestamp, ++mux->sdu_counter, !error);
        }

        if (sdu != NULL) {
            os_mbuf_free_chain(sdu);
        }
    }
}

static void
ble_ll_isoal_mux_flush(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *entry, *prev;
    struct ble_mbuf_hdr *hdr;
    struct os_mbuf *om;
    uint8_t idx;

    prev = NULL;
    entry = STAILQ_FIRST(&mux->pdu_q);
    while (entry != NULL) {
        om = OS_MBUF_PKTHDR_TO_MBUF(entry);

        hdr = BLE_MBUF_HDR_PTR(om);
        idx = pdu_idx_get(hdr);
        if (idx >= mux->bn) {
            /* Pre-transmission - update payload index only */
            pdu_idx_set(hdr, idx - mux->bn);

            prev = entry;
            entry = STAILQ_NEXT(entry, omp_next);
            continue;
        }

        /* Current event data */
        if (prev == NULL) {
            STAILQ_REMOVE_HEAD(&mux->pdu_q, omp_next);
        } else {
            STAILQ_REMOVE_AFTER(&mux->pdu_q, prev, omp_next);
        }

        if (prev == NULL) {
            entry = STAILQ_FIRST(&mux->pdu_q);
        } else {
            entry = STAILQ_NEXT(prev, omp_next);
        }

        os_mbuf_free(om);
    }
}

static void
ble_ll_isoal_mux_event_rx_done(struct ble_ll_isoal_mux *mux)
{
    if (mux->framed) {
        ble_ll_isoal_mux_reassemble(mux);
    } else {
        ble_ll_isoal_mux_recombine(mux);
    }

    ble_ll_isoal_mux_flush(mux);
}

int
ble_ll_isoal_mux_event_done(struct ble_ll_isoal_mux *mux)
{
    struct os_mbuf_pkthdr *pkthdr;
    struct ble_mbuf_hdr *blehdr;
    struct os_mbuf *om;

    pkthdr = STAILQ_FIRST(&mux->sdu_q);
    if (pkthdr) {
        om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);
        blehdr = BLE_MBUF_HDR_PTR(om);
        mux->last_tx_timestamp = mux->event_tx_timestamp;
        mux->last_tx_packet_seq_num = blehdr->txiso.packet_seq_num;
    }

    ble_ll_isoal_mux_event_rx_done(mux);

    if (mux->framed) {
        return ble_ll_isoal_mux_framed_event_done(mux);
    }

    return ble_ll_isoal_mux_unframed_event_done(mux);
}

void
ble_ll_isoal_mux_pdu_enqueue(struct ble_ll_isoal_mux *mux, uint8_t idx, struct os_mbuf *pdu)
{
    struct os_mbuf_pkthdr *entry, *prev;
    struct ble_mbuf_hdr *hdr;
    struct os_mbuf *om;

    hdr = BLE_MBUF_HDR_PTR(pdu);
    pdu_idx_set(hdr, idx);

    prev = NULL;
    entry = STAILQ_FIRST(&mux->pdu_q);
    while (entry) {
        om = OS_MBUF_PKTHDR_TO_MBUF(entry);
        hdr = BLE_MBUF_HDR_PTR(om);

        if (pdu_idx_get(hdr) == idx) {
            /* Already queued */
            os_mbuf_free_chain(pdu);
            return;
        }

        if (pdu_idx_get(hdr) > idx) {
            /* Insert before */
            break;
        }

        prev = entry;
        entry = STAILQ_NEXT(entry, omp_next);
    }

    entry = OS_MBUF_PKTHDR(pdu);
    if (prev) {
        STAILQ_INSERT_AFTER(&mux->pdu_q, prev, entry, omp_next);
    } else {
        STAILQ_INSERT_HEAD(&mux->pdu_q, entry, omp_next);
    }
}

static int
ble_ll_isoal_mux_unframed_get(struct ble_ll_isoal_mux *mux, uint8_t idx,
                              uint8_t *llid, void *dptr)
{
    struct os_mbuf_pkthdr *pkthdr;
    struct os_mbuf *om;
    int32_t rem_len;
    uint8_t sdu_idx;
    uint8_t pdu_idx;
    uint16_t sdu_offset;
    uint8_t pdu_len;

    sdu_idx = idx / mux->pdu_per_sdu;
    pdu_idx = idx - sdu_idx * mux->pdu_per_sdu;

    if (sdu_idx >= mux->sdu_in_event) {
        *llid = 0;
        return 0;
    }

    pkthdr = STAILQ_FIRST(&mux->sdu_q);
    while (pkthdr && sdu_idx--) {
        pkthdr = STAILQ_NEXT(pkthdr, omp_next);
    }

    if (!pkthdr) {
        *llid = 0;
        return 0;
    }

    om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);
    sdu_offset = pdu_idx * mux->max_pdu;
    rem_len = OS_MBUF_PKTLEN(om) - sdu_offset;

    if (OS_MBUF_PKTLEN(om) == 0) {
        /* LLID = 0b00: Zero-Length SDU (complete SDU) */
        *llid = 0;
        pdu_len = 0;
    } else if (rem_len <= 0) {
        /* LLID = 0b01: ISO Data PDU used as padding */
        *llid = 1;
        pdu_len = 0;
    } else {
        /* LLID = 0b00: Data remaining fits the ISO Data PDU size,
         *              it's end fragment of an SDU or complete SDU.
         * LLID = 0b01: Data remaining exceeds the ISO Data PDU size,
         *              it's start or continuation fragment of an SDU.
         */
        *llid = rem_len > mux->max_pdu;
        pdu_len = min(mux->max_pdu, rem_len);
    }

    os_mbuf_copydata(om, sdu_offset, pdu_len, dptr);

    return pdu_len;
}

static int
ble_ll_isoal_mux_framed_get(struct ble_ll_isoal_mux *mux, uint8_t idx,
                            uint8_t *llid, uint8_t *dptr)
{
    struct ble_mbuf_hdr *blehdr;
    struct os_mbuf_pkthdr *pkthdr;
    struct os_mbuf *om;
    uint32_t time_offset;
    uint16_t seghdr;
    uint16_t rem_len = 0;
    uint16_t sdu_offset = 0;
    uint8_t num_sdu;
    uint8_t num_pdu;
    uint8_t frag_len;
    uint8_t pdu_offset = 0;
    bool sc = mux->sc;
    uint8_t hdr_len = 0;

    *llid = 0b10;

    num_sdu = mux->sdu_in_event;
    if (num_sdu == 0) {
        return 0;
    }

    num_pdu = idx;

    /* Skip the idx PDUs */
    pkthdr = STAILQ_FIRST(&mux->sdu_q);
    while (pkthdr && num_sdu > 0 && num_pdu > 0) {
        om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);

        rem_len = OS_MBUF_PKTLEN(om) - sdu_offset;
        hdr_len = sc ? 2 /* Segmentation Header */
                     : 5 /* Segmentation Header + TimeOffset */;

        if (mux->max_pdu <= hdr_len + pdu_offset) {
            /* Advance to next PDU */
            pdu_offset = 0;
            num_pdu--;
            continue;
        }

        frag_len = min(rem_len, mux->max_pdu - hdr_len - pdu_offset);

        pdu_offset += hdr_len + frag_len;

        if (frag_len == rem_len) {
            /* Process next SDU */
            sdu_offset = 0;
            num_sdu--;
            pkthdr = STAILQ_NEXT(pkthdr, omp_next);

            sc = 0;
        } else {
            sdu_offset += frag_len;

            sc = 1;
        }
    }

    if (num_pdu > 0) {
        return 0;
    }

    BLE_LL_ASSERT(pdu_offset == 0);

    while (pkthdr && num_sdu > 0) {
        om = OS_MBUF_PKTHDR_TO_MBUF(pkthdr);

        rem_len = OS_MBUF_PKTLEN(om) - sdu_offset;
        hdr_len = sc ? 2 /* Segmentation Header */
                     : 5 /* Segmentation Header + TimeOffset */;

        if (mux->max_pdu <= hdr_len + pdu_offset) {
            break;
        }

        frag_len = min(rem_len, mux->max_pdu - hdr_len - pdu_offset);

        /* Segmentation Header */
        seghdr = BLE_LL_ISOAL_SEGHDR(sc, frag_len == rem_len, frag_len + hdr_len - 2);
        put_le16(dptr + pdu_offset, seghdr);
        pdu_offset += 2;

        /* Time Offset */
        if (hdr_len > 2) {
            blehdr = BLE_MBUF_HDR_PTR(om);

            time_offset = mux->event_tx_timestamp -
                          blehdr->txiso.cpu_timestamp;
            put_le24(dptr + pdu_offset, time_offset);
            pdu_offset += 3;
        }

        /* ISO Data Fragment */
        os_mbuf_copydata(om, sdu_offset, frag_len, dptr + pdu_offset);
        pdu_offset += frag_len;

        if (frag_len == rem_len) {
            /* Process next SDU */
            sdu_offset = 0;
            num_sdu--;
            pkthdr = STAILQ_NEXT(pkthdr, omp_next);

            sc = 0;
        } else {
            sdu_offset += frag_len;

            sc = 1;
        }
    }

    return pdu_offset;
}

int
ble_ll_isoal_mux_pdu_get(struct ble_ll_isoal_mux *mux, uint8_t idx,
                         uint8_t *llid, void *dptr)
{
    if (mux->framed) {
        return ble_ll_isoal_mux_framed_get(mux, idx, llid, dptr);
    }

    return ble_ll_isoal_mux_unframed_get(mux, idx, llid, dptr);
}

int
ble_ll_isoal_mux_cb_set(struct ble_ll_isoal_mux *mux,
                        const struct ble_ll_isoal_mux_cb *cb)
{
    mux->cb = cb;

    return 0;
}

void
ble_ll_isoal_init(void)
{
}

void
ble_ll_isoal_reset(void)
{
}

#endif /* BLE_LL_ISO */
