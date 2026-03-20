/*
 * g72x.h — Standalone G.721 ADPCM codec for DeDECTive
 *
 * Based on the public-domain implementation by Sun Microsystems, Inc.
 * Original source provided for unrestricted use without charge.
 *
 * This file contains only the decoder path needed for DECT voice.
 */
#ifndef DEDECTIVE_G72X_H
#define DEDECTIVE_G72X_H

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_ENCODING_LINEAR 3

/*
 * Internal state for the G.721 ADPCM codec.
 * Must be initialized with g72x_init_state() before first use.
 * Maintains prediction filter state between successive calls.
 */
struct g72x_state {
    long  yl;    /* Locked or steady state step size multiplier        */
    short yu;    /* Unlocked or non-steady state step size multiplier  */
    short dms;   /* Short term energy estimate                         */
    short dml;   /* Long term energy estimate                          */
    short ap;    /* Linear weighting coefficient of 'yl' and 'yu'     */
    short a[2];  /* Coefficients of pole portion of prediction filter  */
    short b[6];  /* Coefficients of zero portion of prediction filter  */
    short pk[2]; /* Signs of previous two partially reconstructed samples */
    short dq[6]; /* Previous 6 quantized difference samples (float fmt) */
    short sr[2]; /* Previous 2 reconstructed samples (float fmt)       */
    char  td;    /* Delayed tone detect                                */
};

/* Initialize codec state. Call before first decode. */
void g72x_init_state(struct g72x_state *state_ptr);

/*
 * Decode one 4-bit G.721 ADPCM code to a 16-bit linear PCM sample.
 *
 *   code       – 4-bit ADPCM code (0–15)
 *   out_coding – must be AUDIO_ENCODING_LINEAR (3)
 *   state_ptr  – codec state (updated in place)
 *
 * Returns the decoded 16-bit PCM sample value.
 */
int g721_decoder(int code, int out_coding, struct g72x_state *state_ptr);

#ifdef __cplusplus
}
#endif

#endif /* DEDECTIVE_G72X_H */
