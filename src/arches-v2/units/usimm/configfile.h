#ifndef __CONFIG_FILE_IN_H__
#define __CONFIG_FILE_IN_H__

#include "params.h"
#include <utility>
#include <stdio.h>

// We must use ENUM over #define otherwise we run into error with clang++ and boost: redefining CR
enum
{
    EOL   = 10,
    CR    = 13,
    SPACE = 32,
    TAB   = 9
};

//#define 	EOL 	10
//#define 	CR 	13
//#define 	SPACE	32
//#define		TAB	9

typedef enum
{
    processor_clk_multiplier_token,
    robsize_token,
    max_retire_token,
    max_fetch_token,
    pipelinedepth_token,

    num_channels_token,
    num_ranks_token,
    num_banks_token,
    num_rows_token,
    num_columns_token,
    cache_line_size_token,
    address_bits_token,

    dram_clk_frequency_token,
    t_rcd_token,
    t_rp_token,
    t_cas_token,
    t_rc_token,
    t_ras_token,
    t_rrd_token,
    t_faw_token,
    t_wr_token,
    t_wtr_token,
    t_rtp_token,
    t_ccd_token,
    t_rfc_token,
    t_refi_token,
    t_cwd_token,
    t_rtrs_token,
    t_pd_min_token,
    t_xp_token,
    t_xp_dll_token,
    t_data_trans_token,

    vdd_token,
    idd0_token,
    idd2p0_token,
    idd2p1_token,
    idd2n_token,
    idd3p_token,
    idd3n_token,
    idd4r_token,
    idd4w_token,
    idd5_token,

    wq_capacity_token,
    address_mapping_token,
    wq_lookup_latency_token,

    comment_token,
    unknown_token
} token_t;


// Helper to parse which token was read in
typedef struct
{
    const char *keyword;
    token_t     token;
    bool        isInteger;
} tokenTranslator;

const tokenTranslator tokenTranslatorTable[] =
{
    {"//",                          comment_token,                  true},
    {"PROCESSOR_CLK_MULTIPLIER",    processor_clk_multiplier_token, true},
    {"ROBSIZE",                     robsize_token,                  true},
    {"MAX_RETIRE",                  max_retire_token,               true},
    {"MAX_FETCH",                   max_fetch_token,                true},
    {"PIPELINEDEPTH",               pipelinedepth_token,            true},

    {"NUM_CHANNELS",                num_channels_token,             true},
    {"NUM_RANKS",                   num_ranks_token,                true},
    {"NUM_BANKS",                   num_banks_token,                true},
    {"NUM_ROWS",                    num_rows_token,                 true},
    {"NUM_COLUMNS",                 num_columns_token,              true},
    {"CACHE_LINE_SIZE",             cache_line_size_token,          true},
    {"ADDRESS_BITS",                address_bits_token,             true},

    {"DRAM_CLK_FREQUENCY",          dram_clk_frequency_token,       true},
    {"T_RC",                        t_rc_token,                     true},
    {"T_RP",                        t_rp_token,                     true},
    {"T_CAS",                       t_cas_token,                    true},
    {"T_RCD",                       t_rcd_token,                    true},
    {"T_RAS",                       t_ras_token,                    true},
    {"T_RRD",                       t_rrd_token,                    true},
    {"T_FAW",                       t_faw_token,                    true},
    {"T_WR",                        t_wr_token,                     true},
    {"T_WTR",                       t_wtr_token,                    true},
    {"T_RTP",                       t_rtp_token,                    true},
    {"T_CCD",                       t_ccd_token,                    true},
    {"T_RFC",                       t_rfc_token,                    true},
    {"T_REFI",                      t_refi_token,                   true},
    {"T_CWD",                       t_cwd_token,                    true},
    {"T_RTRS",                      t_rtrs_token,                   true},
    {"T_PD_MIN",                    t_pd_min_token,                 true},
    {"T_XP",                        t_xp_token,                     true},
    {"T_XP_DLL",                    t_xp_dll_token,                 true},
    {"T_DATA_TRANS",                t_data_trans_token,             true},

    {"VDD",                         vdd_token,                      false},
    {"IDD0",                        idd0_token,                     false},
    {"IDD2P0",                      idd2p0_token,                   false},
    {"IDD2P1",                      idd2p1_token,                   false},
    {"IDD2N",                       idd2n_token,                    false},
    {"IDD3P",                       idd3p_token,                    false},
    {"IDD3N",                       idd3n_token,                    false},
    {"IDD4R",                       idd4r_token,                    false},
    {"IDD4W",                       idd4w_token,                    false},
    {"IDD5",                        idd5_token,                     false},

    {"WQ_CAPACITY",                 wq_capacity_token,              true},
    {"ADDRESS_MAPPING",             address_mapping_token,          true},
    {"WQ_LOOKUP_LATENCY",           wq_lookup_latency_token,        true}
};



token_t tokenize(const char * input, bool &isInteger) {
    const size_t numTokens = sizeof(tokenTranslatorTable) / sizeof(tokenTranslator);

    const size_t lengthInput = strlen(input);
    for(size_t i = 0; i < numTokens; ++i) {
        // Special case for comments which are not separated by a space, aka //BLOB
        const size_t lengthToCompare = (tokenTranslatorTable[i].token != comment_token ? lengthInput : strlen(tokenTranslatorTable[i].keyword));
        if(strncmp(input, tokenTranslatorTable[i].keyword, lengthToCompare) == 0) {
            isInteger = tokenTranslatorTable[i].isInteger;
            return tokenTranslatorTable[i].token;
        }
    }

    // Unknown token
    printf("PANIC : Unknown token %s\n",input);
    return unknown_token;
}


void read_config_file(FILE * fin)
{
    char  c;
    char  input_string[256];
    int   input_int;
    float input_float;

    while ((c = fgetc(fin)) != EOF)
    {
        if ((c != EOL)   &&
            (c != CR)    &&
            (c != SPACE) &&
            (c != TAB))
        {
            if (fscanf(fin, "%s", &input_string[1]) == 0)
                continue;
            input_string[0] = c;
        }
        else
        {
            if (fscanf(fin, "%s", &input_string[0]) == 0)
                continue;
        }
        bool loadAsInt;
        const token_t input_field = tokenize(&input_string[0], loadAsInt);

        // badness
        if (input_field == unknown_token)
        {
            printf("PANIC: bad token (%s) in cfg file\n", input_string);
            continue;
        }

        // Ignore comment
        if (input_field == comment_token)
        {
            while (((c = fgetc(fin)) != EOL) && (c != EOF))
            {
                /*comment, to be ignored */
            }
            continue;
        }

        // Attempt to read the data into correct type
        if (loadAsInt)
        {
            if (fscanf(fin, "%d", &input_int) == 0)
            {
                printf("PANIC : token %s must have integer parameter value\n", input_string);
                continue;
            }
        }
        else
        {
            if (fscanf(fin, "%f", &input_float) == 0)
            {
                printf("PANIC : token %s must have float parameter value\n", input_string);
                continue;
            }
        }

        // Apply value correctly
        switch (input_field)
        {
            case comment_token:
            case unknown_token:
                // already handled above
                break;

            case processor_clk_multiplier_token:    PROCESSOR_CLK_MULTIPLIER = input_int;                               break;
            case robsize_token:                     ROBSIZE                  = input_int;                               break;
            case max_retire_token:                  MAX_RETIRE               = input_int;                               break;
            case max_fetch_token:                   MAX_FETCH                = input_int;                               break;
            case pipelinedepth_token:               PIPELINEDEPTH            = input_int;                               break;

            case num_channels_token:                NUM_CHANNELS             = input_int;                               break;
            case num_ranks_token:                   NUM_RANKS                = input_int;                               break;
            case num_banks_token:                   NUM_BANKS                = input_int;                               break;
            case num_rows_token:                    NUM_ROWS                 = input_int;                               break;
            case num_columns_token:                 NUM_COLUMNS              = input_int;                               break;
            case cache_line_size_token:             CACHE_LINE_SIZE          = input_int;                               break;
            case address_bits_token:                ADDRESS_BITS             = input_int;                               break;

            case dram_clk_frequency_token:          DRAM_CLK_FREQUENCY       = input_int;                               break;
            case t_rcd_token:                       T_RCD                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_rp_token:                        T_RP                     = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_cas_token:                       T_CAS                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_rc_token:                        T_RC                     = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_ras_token:                       T_RAS                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_rrd_token:                       T_RRD                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_faw_token:                       T_FAW                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_wr_token:                        T_WR                     = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_wtr_token:                       T_WTR                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_rtp_token:                       T_RTP                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_ccd_token:                       T_CCD                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_rfc_token:                       T_RFC                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_refi_token:                      T_REFI                   = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_cwd_token:                       T_CWD                    = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_rtrs_token:                      T_RTRS                   = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_pd_min_token:                    T_PD_MIN                 = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_xp_token:                        T_XP                     = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_xp_dll_token:                    T_XP_DLL                 = input_int * PROCESSOR_CLK_MULTIPLIER;    break;
            case t_data_trans_token:                T_DATA_TRANS             = input_int * PROCESSOR_CLK_MULTIPLIER;    break;

            case vdd_token:                         VDD                      = input_float;                             break;
            case idd0_token:                        IDD0                     = input_float;                             break;
            case idd2p0_token:                      IDD2P0                   = input_float;                             break;
            case idd2p1_token:                      IDD2P1                   = input_float;                             break;
            case idd2n_token:                       IDD2N                    = input_float;                             break;
            case idd3p_token:                       IDD3P                    = input_float;                             break;
            case idd3n_token:                       IDD3N                    = input_float;                             break;
            case idd4r_token:                       IDD4R                    = input_float;                             break;
            case idd4w_token:                       IDD4W                    = input_float;                             break;
            case idd5_token:                        IDD5                     = input_float;                             break;

            case wq_capacity_token:                 WQ_CAPACITY              = input_int;                               break;
            case address_mapping_token:             ADDRESS_MAPPING          = input_int;                               break;
            case wq_lookup_latency_token:           WQ_LOOKUP_LATENCY        = input_int;                               break;

            // badness
            default:
                printf("PANIC: bad token (%s) in cfg file\n", input_string);
                break;
        }
    }
}


void print_params()
{
    printf("----------------------------------------------------------------------------------------\n");
    printf("------------------------\n");
    printf("- SIMULATOR PARAMETERS -\n");
    printf("------------------------\n");
    printf("\n-------------\n");
    printf("- PROCESSOR -\n");
    printf("-------------\n");
    printf("PROCESSOR_CLK_MULTIPLIER:   %d\n",  PROCESSOR_CLK_MULTIPLIER);
    printf("ROBSIZE:                    %6d\n", ROBSIZE);
    printf("MAX_FETCH:                  %6d\n", MAX_FETCH);
    printf("MAX_RETIRE:                 %6d\n", MAX_RETIRE);
    printf("PIPELINEDEPTH:              %6d\n", PIPELINEDEPTH);

    printf("\n---------------\n");
    printf("- DRAM Config -\n");
    printf("---------------\n");
    printf("NUM_CHANNELS:               %6d\n", NUM_CHANNELS);
    printf("NUM_RANKS:                  %6d\n", NUM_RANKS);
    printf("NUM_BANKS:                  %6d\n", NUM_BANKS);
    printf("NUM_ROWS:                   %6d\n", NUM_ROWS);
    printf("NUM_COLUMNS:                %6d\n", NUM_COLUMNS);
    printf("CACHE_LINE_SIZE:            %6d\n", CACHE_LINE_SIZE);
    printf("ADDRESS_BITS:               %6d\n", ADDRESS_BITS);

    printf("\n---------------\n");
    printf("- DRAM Timing -\n");
    printf("---------------\n");
    printf("T_RCD:                      %6d\n", T_RCD);
    printf("T_RP:                       %6d\n", T_RP);
    printf("T_CAS:                      %6d\n", T_CAS);
    printf("T_RC:                       %6d\n", T_RC);
    printf("T_RAS:                      %6d\n", T_RAS);
    printf("T_RRD:                      %6d\n", T_RRD);
    printf("T_FAW:                      %6d\n", T_FAW);
    printf("T_WR:                       %6d\n", T_WR);
    printf("T_WTR:                      %6d\n", T_WTR);
    printf("T_RTP:                      %6d\n", T_RTP);
    printf("T_CCD:                      %6d\n", T_CCD);
    printf("T_RFC:                      %6d\n", T_RFC);
    printf("T_REFI:                     %6d\n", T_REFI);
    printf("T_CWD:                      %6d\n", T_CWD);
    printf("T_RTRS:                     %6d\n", T_RTRS);
    printf("T_PD_MIN:                   %6d\n", T_PD_MIN);
    printf("T_XP:                       %6d\n", T_XP);
    printf("T_XP_DLL:                   %6d\n", T_XP_DLL);
    printf("T_DATA_TRANS:               %6d\n", T_DATA_TRANS);

    printf("\n---------------------------\n");
    printf("- DRAM Idd Specifications -\n");
    printf("---------------------------\n");

    printf("VDD:                        %05.2f\n", VDD);
    printf("IDD0:                       %05.2f\n", IDD0);
    printf("IDD2P0:                     %05.2f\n", IDD2P0);
    printf("IDD2P1:                     %05.2f\n", IDD2P1);
    printf("IDD2N:                      %05.2f\n", IDD2N);
    printf("IDD3P:                      %05.2f\n", IDD3P);
    printf("IDD3N:                      %05.2f\n", IDD3N);
    printf("IDD4R:                      %05.2f\n", IDD4R);
    printf("IDD4W:                      %05.2f\n", IDD4W);
    printf("IDD5:                       %05.2f\n", IDD5);

    printf("\n-------------------\n");
    printf("- DRAM Controller -\n");
    printf("-------------------\n");
    printf("WQ_CAPACITY:                %6d\n", WQ_CAPACITY);
    printf("ADDRESS_MAPPING:            %6d\n", ADDRESS_MAPPING);
    printf("WQ_LOOKUP_LATENCY:          %6d\n", WQ_LOOKUP_LATENCY);
    printf("\n----------------------------------------------------------------------------------------\n");
}


#endif // __CONFIG_FILE_IN_H__
