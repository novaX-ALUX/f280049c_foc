#include "check.h"
#include "dronecan_param.h"
#include "dronecan.h"
#include "nvparam.h"
#include <string.h>

#include "dronecan_param_golden.inc"

#define NCODEC ((int)(sizeof(GOLD_GS_CODEC) / sizeof(GOLD_GS_CODEC[0])))
#define NTX    ((int)(sizeof(GOLD_GS_TX) / sizeof(GOLD_GS_TX[0])))

/* Run codec golden case `gi`, asserting the response matches byte-for-byte and persist flag. */
static void run_codec(int gi, const char *want_name, nvparam_t *nv, bool exp_persist)
{
    uint16_t out[DRONECAN_PARAM_RESP_MAX];
    bool persist = true;
    const gold_gs_codec_t *g = &GOLD_GS_CODEC[gi];
    uint16_t n, i;

    CHECK(strcmp(g->name, want_name) == 0);   /* golden ordering sanity */
    n = dronecan_param_build_response(nv, g->req, g->req_len, out,
                                      DRONECAN_PARAM_RESP_MAX, &persist);
    CHECK(n == g->resp_len);
    for (i = 0; i < g->resp_len; i++) {
        CHECK((out[i] & 0xFFu) == (g->resp[i] & 0xFFu));
    }
    CHECK(persist == exp_persist);
}

int main(void)
{
    /* table sanity */
    CHECK(dronecan_param_count() == 3u);
    CHECK(strcmp(dronecan_param_name(0), "node_id") == 0);
    CHECK(strcmp(dronecan_param_name(1), "park_ref_valid") == 0);
    CHECK(strcmp(dronecan_param_name(2), "park_ref_target_rev") == 0);
    CHECK(dronecan_param_name(3) == NULL);

    /* 0: GET node_id by name, node_id=0 -> no write */
    { nvparam_t nv; nvparam_set_defaults(&nv);
      run_codec(0, "get_nodeid_byname_0", &nv, false);
      CHECK(nv.node_id == 0u); }

    /* 1: GET node_id by index, node_id=42 -> readback 42, no write */
    { nvparam_t nv; nvparam_set_defaults(&nv); nv.node_id = 42u;
      run_codec(1, "get_nodeid_byidx_42", &nv, false);
      CHECK(nv.node_id == 42u); }

    /* 2: SET node_id=25 (0 -> 25) -> persist */
    { nvparam_t nv; nvparam_set_defaults(&nv);
      run_codec(2, "set_nodeid_25", &nv, true);
      CHECK(nv.node_id == 25u); }

    /* 3: SET node_id=200 (illegal) on a node currently 5 -> nvparam clamps to DNA(0), persist */
    { nvparam_t nv; nvparam_set_defaults(&nv); nv.node_id = 5u;
      run_codec(3, "set_nodeid_bad_200", &nv, true);
      CHECK(nv.node_id == 0u); }

    /* 4: GET park_ref_valid (false) -> no write */
    { nvparam_t nv; nvparam_set_defaults(&nv);
      run_codec(4, "get_parkvalid", &nv, false);
      CHECK(!nv.park_ref_valid); }

    /* 5: SET park_ref_valid=1 (false -> true) -> persist */
    { nvparam_t nv; nvparam_set_defaults(&nv);
      run_codec(5, "set_parkvalid_1", &nv, true);
      CHECK(nv.park_ref_valid); }

    /* 6: SET park_ref_target_rev=0.25 while valid (sticks) -> persist */
    { nvparam_t nv; nvparam_set_defaults(&nv);
      nvparam_update_park_ref(&nv, true, 0.0f);
      run_codec(6, "set_parktgt_025", &nv, true);
      CHECK(nv.park_ref_valid);
      CHECK(nv.park_ref_target_rev > 0.249f && nv.park_ref_target_rev < 0.251f); }

    /* 7: SET park_ref_target_rev=NaN -> nvparam rejects + invalidates the ref, persist */
    { nvparam_t nv; nvparam_set_defaults(&nv);
      nvparam_update_park_ref(&nv, true, 0.5f);
      run_codec(7, "set_parktgt_nan", &nv, true);
      CHECK(!nv.park_ref_valid);
      CHECK(nv.park_ref_target_rev == 0.0f); }

    /* 8: GET unknown name -> empty response, no write */
    { nvparam_t nv; nvparam_set_defaults(&nv); nv.node_id = 9u;
      run_codec(8, "get_unknown_name", &nv, false);
      CHECK(nv.node_id == 9u); }

    /* 9: GET unknown index -> empty response, no write */
    { nvparam_t nv; nvparam_set_defaults(&nv); nv.node_id = 9u;
      run_codec(9, "get_unknown_idx_99", &nv, false);
      CHECK(nv.node_id == 9u); }

    /* nv == NULL: every request yields the canonical empty response, never persists. */
    { uint16_t out[DRONECAN_PARAM_RESP_MAX]; bool persist = true;
      uint16_t n = dronecan_param_build_response(NULL, GOLD_GS_CODEC[0].req,
                       GOLD_GS_CODEC[0].req_len, out, DRONECAN_PARAM_RESP_MAX, &persist);
      CHECK(n == 4u);                 /* empty value + 3 empty numerics, empty name */
      CHECK(!persist); }

    CHECK(NCODEC == 10);

    /* Malformed: claims an int64 value but only 2 bytes present -> value dropped, treated as a
       GET of index 0 (node_id); never reads past the buffer, never persists. */
    { nvparam_t nv; nvparam_set_defaults(&nv); nv.node_id = 7u;
      uint16_t req[2] = { 0x00u, 0x01u };   /* index=0, value tag=integer, but no payload */
      uint16_t out[DRONECAN_PARAM_RESP_MAX]; bool persist = true;
      uint16_t n = dronecan_param_build_response(&nv, req, 2u, out,
                                                 DRONECAN_PARAM_RESP_MAX, &persist);
      CHECK(n == 43u);            /* node_id response */
      CHECK(out[0] == 0x01u && out[1] == 0x07u);  /* value = int(7), unchanged */
      CHECK(!persist);
      CHECK(nv.node_id == 7u); }

    /* ===== TRANSPORT: request frames in -> response frames out (reassembly + framing) ===== */
    {
        int c;
        for (c = 0; c < NTX; c++) {
            const gold_gs_tx_t *g = &GOLD_GS_TX[c];
            nvparam_t nv;
            dronecan_t dn;
            dronecan_cfg_t cfg;
            dronecan_rx_result_t res;
            dronecan_frame_t out[32];
            int n, i, k;

            nvparam_set_defaults(&nv);
            memset(&cfg, 0, sizeof(cfg));
            cfg.node_id = g->our_node;        /* allocated (static) so the node can answer */
            cfg.esc_index = 0u;
            cfg.nvparam = &nv;
            dronecan_init(&dn, &cfg);
            CHECK(dronecan_node_id(&dn) == g->our_node);

            /* feed the request frame(s) */
            for (i = 0; i < g->n_req; i++) {
                dronecan_frame_t f;
                f.extended = true;
                f.id = g->req_id[i];
                f.dlc = g->req_dlc[i];
                for (k = 0; k < 8; k++) { f.data[k] = g->req_data[i][k]; }
                dronecan_on_rx(&dn, &f, &res);
            }

            /* a Set must mark the param dirty; a pure Get must not */
            if (strstr(g->name, "set_") != NULL) {
                CHECK(dronecan_param_dirty(&dn));
            } else {
                CHECK(!dronecan_param_dirty(&dn));
            }

            /* tick emits the GetSet response FIRST (before NodeStatus/esc.Status) */
            {
                esc_telemetry_t tel; memset(&tel, 0, sizeof(tel));
                n = dronecan_tick(&dn, 1000u, &tel, out, 32);
            }
            CHECK(n >= g->n_resp);
            for (i = 0; i < g->n_resp; i++) {
                CHECK(out[i].id == g->resp_id[i]);
                CHECK(out[i].dlc == g->resp_dlc[i]);
                for (k = 0; k < (int)g->resp_dlc[i]; k++) {
                    CHECK((out[i].data[k] & 0xFFu) == (g->resp_data[i][k] & 0xFFu));
                }
            }
            /* response consumed -> not pending anymore */
            CHECK(!dn.gs_pending);
        }
        CHECK(NTX == 2);
    }

    CHECK_DONE();
}
