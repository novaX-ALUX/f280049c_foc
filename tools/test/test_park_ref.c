#include "check.h"
#include "park_ref.h"

static park_ref_cfg_t base_cfg(void)
{
    park_ref_cfg_t c = {0};
    c.speed_thresh_krpm    = 0.01f;
    c.enc_vel_thresh_revps = 0.02f;
    c.throttle_eps         = 0.01f;
    c.learn_hold_s         = 0.10f;
    return c;
}

/* NV stub: configurable validity + value. */
static bool  g_nv_valid;
static float g_nv_value;
static float nv_load(void *ctx, bool *out_valid)
{
    (void)ctx;
    *out_valid = g_nv_valid;
    return g_nv_value;
}

int main(void)
{
    /* NV invalid -> unlearned; strict gate; sustained still -> capture + store request. */
    {
        g_nv_valid = false;
        g_nv_value = 0.0f;
        park_ref_cfg_t c = base_cfg();
        park_ref_state_t st;
        park_ref_init(&st, &c, nv_load, NULL);
        CHECK(!park_ref_valid(&st));

        /* Not yet held long enough. */
        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, true, 0.42f, 0.05f);
        CHECK(!park_ref_valid(&st));
        CHECK(!st.needs_store);

        /* Cross the hold threshold -> capture current angle + raise store request. */
        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, true, 0.42f, 0.06f);
        CHECK(park_ref_valid(&st));
        CHECK(st.needs_store);
        CHECK_NEAR(park_ref_target(&st), 0.42f, 1e-4f);
        CHECK_NEAR(st.new_target, 0.42f, 1e-4f);

        /* App persists, then clears the request. */
        park_ref_clear_store_request(&st);
        CHECK(!st.needs_store);
    }

    /* Disturbance mid-dwell resets the timer: never captures. */
    {
        g_nv_valid = false;
        park_ref_cfg_t c = base_cfg();
        park_ref_state_t st;
        park_ref_init(&st, &c, nv_load, NULL);

        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, true, 0.42f, 0.05f); /* dwell 0.05 */
        park_ref_update(&st, true, false, 0.0f, 0.5f, 0.0f, true, 0.42f, 0.05f); /* spinning -> reset */
        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, true, 0.42f, 0.05f); /* dwell 0.05 again */
        CHECK(!park_ref_valid(&st)); /* never reached 0.10 contiguous */
    }

    /* Gate blocked while armed / throttle up / encoder invalid. */
    {
        g_nv_valid = false;
        park_ref_cfg_t c = base_cfg();
        park_ref_state_t st;
        park_ref_init(&st, &c, nv_load, NULL);
        park_ref_update(&st, true, true,  0.0f, 0.0f, 0.0f, true,  0.42f, 0.2f); /* armed */
        CHECK(!park_ref_valid(&st));
        park_ref_update(&st, true, false, 0.5f, 0.0f, 0.0f, true,  0.42f, 0.2f); /* throttle */
        CHECK(!park_ref_valid(&st));
        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, false, 0.42f, 0.2f); /* enc invalid */
        CHECK(!park_ref_valid(&st));
        park_ref_update(&st, false, false, 0.0f, 0.0f, 0.0f, true, 0.42f, 0.2f); /* not disarmed */
        CHECK(!park_ref_valid(&st));
    }

    /* NV valid at boot -> use it directly, no re-learn, no store request. */
    {
        g_nv_valid = true;
        g_nv_value = 0.123f;
        park_ref_cfg_t c = base_cfg();
        park_ref_state_t st;
        park_ref_init(&st, &c, nv_load, NULL);
        CHECK(park_ref_valid(&st));
        CHECK_NEAR(park_ref_target(&st), 0.123f, 1e-4f);
        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, true, 0.42f, 0.5f);
        CHECK(!st.needs_store);
        CHECK_NEAR(park_ref_target(&st), 0.123f, 1e-4f); /* unchanged */
    }

    /* invalidate -> re-learn possible. */
    {
        g_nv_valid = true;
        g_nv_value = 0.123f;
        park_ref_cfg_t c = base_cfg();
        park_ref_state_t st;
        park_ref_init(&st, &c, nv_load, NULL);
        park_ref_invalidate(&st);
        CHECK(!park_ref_valid(&st));
        park_ref_update(&st, true, false, 0.0f, 0.0f, 0.0f, true, 0.30f, 0.2f); /* > hold */
        CHECK(park_ref_valid(&st));
        CHECK_NEAR(park_ref_target(&st), 0.30f, 1e-4f);
    }

    CHECK_DONE();
}
