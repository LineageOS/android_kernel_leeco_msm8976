
#ifndef _TYPE_C_INFO_H_
#define _TYPE_C_INFO_H_

enum {
    TYPE_C_DIR_TOP= 0,
    TYPE_C_DIR_BOTTOM = 1,
};

enum {
    TYPE_C_MODE_MAINTAIN= 0,
    TYPE_C_MODE_UFP = 1,
    TYPE_C_MODE_DFP = 2,
    TYPE_C_MODE_DRP = 3,
};

enum {
    TYPEC_AUDIO_DIGITAL = 0,
    TYPEC_AUDIO_ANALOG = 1,
};

struct typec_info_data {
    u8 typec_dir;
    u8 typec_mode;
    u8 typec_mode_result;
    u8 typec_notif_ready;
};

extern bool is_nxp_type_c_register;
extern bool is_ti_type_c_register;
extern int letv_audio_mode_supported(void *data);
extern void cclogic_set_audio_mode(bool mode);

#endif /* _TYPE_C_INFO_H_ */
