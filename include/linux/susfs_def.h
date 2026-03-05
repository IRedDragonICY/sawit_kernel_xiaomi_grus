#ifndef KSU_SUSFS_DEF_H
#define KSU_SUSFS_DEF_H

/********/
/* ENUM */
/********/
/* shared with userspace ksu_susfs tool */
#define SUSFS_MAGIC 0xFAFAFAFA

#define SUSFS_VERSION "2.0.0"
#define SUSFS_VARIANT "KSU_NEXT"

#define CMD_SUSFS_SHOW_VERSION 0x555e1
#define CMD_SUSFS_SHOW_ENABLED_FEATURES 0x555e2
#define CMD_SUSFS_SHOW_VARIANT 0x555e3
#define CMD_SUSFS_ENABLE_AVC_LOG_SPOOFING 0x60010

#define SUSFS_ENABLED_FEATURES_SIZE 8192
#define SUSFS_MAX_VERSION_BUFSIZE 16
#define SUSFS_MAX_VARIANT_BUFSIZE 16

struct st_susfs_version {
  char susfs_version[SUSFS_MAX_VERSION_BUFSIZE];
  int err;
};

struct st_susfs_variant {
  char susfs_variant[SUSFS_MAX_VARIANT_BUFSIZE];
  int err;
};

struct st_susfs_enabled_features {
  char enabled_features[SUSFS_ENABLED_FEATURES_SIZE];
  int err;
};

struct st_susfs_avc_log_spoofing {
  int enabled;
  int err;
};

/* function declarations for version/variant/features reporting */
void susfs_show_version(void __user **user_info);
void susfs_show_variant(void __user **user_info);
void susfs_get_enabled_features(void __user **user_info);
void susfs_set_avc_log_spoofing(void __user **user_info);

#endif
