/*
 * fs/sdcardfs/main.c
 *
 * Copyright (c) 2013 Samsung Electronics Co. Ltd
 *   Authors: Daeho Jeong, Woojoong Lee, Seunghwan Hyun,
 *               Sunghwan Yun, Sungjong Seo
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by
 *
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009     Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include "sdcardfs.h"
#include <linux/module.h>
#include <linux/types.h>
#include <linux/parser.h>

enum {
	Opt_uid,
	Opt_gid,
	Opt_wgid,
	Opt_debug,
	Opt_split,
	Opt_derive,
	Opt_lower_fs,
	Opt_reserved_mb,
	Opt_err,
};

static const match_table_t sdcardfs_tokens = {
	{Opt_uid, "uid=%u"},
	{Opt_gid, "gid=%u"},
	{Opt_wgid, "wgid=%u"},
	{Opt_debug, "debug"},
	{Opt_split, "split"},
	{Opt_derive, "derive=%s"},
	{Opt_lower_fs, "lower_fs=%s"},
	{Opt_reserved_mb, "reserved_mb=%u"},
	{Opt_err, NULL}
};

static int parse_options(struct super_block *sb, char *options, int silent,
				int *debug, struct sdcardfs_mount_options *opts)
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;
	char *string_option;

	/* by default, we use AID_MEDIA_RW as uid, gid */
	opts->fs_low_uid = AID_MEDIA_RW;
	opts->fs_low_gid = AID_MEDIA_RW;
	/* by default, we use AID_SDCARD_RW as write_gid */
	opts->write_gid = AID_SDCARD_RW;
	/* default permission policy
	 * (DERIVE_NONE | DERIVE_LEGACY | DERIVE_UNIFIED) */
	opts->derive = DERIVE_NONE;
	opts->split_perms = 0;
	/* by default, we use LOWER_FS_EXT4 as lower fs type */
	opts->lower_fs = LOWER_FS_EXT4;
	/* by default, 0MB is reserved */
	opts->reserved_mb = 0;

	*debug = 0;

	if (!options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;
		if (!*p)
			continue;

		token = match_token(p, sdcardfs_tokens, args);

		switch (token) {
		case Opt_debug:
			*debug = 1;
			break;
		case Opt_uid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_uid = option;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_gid = option;
			break;
		case Opt_wgid:
			if (match_int(&args[0], &option))
				return 0;
			opts->write_gid = option;
			break;
		case Opt_split:
			opts->split_perms=1;
			break;
		case Opt_derive:
			string_option = match_strdup(&args[0]);
			if (!strcmp("none", string_option)) {
				opts->derive = DERIVE_NONE;
			} else if (!strcmp("legacy", string_option)) {
				opts->derive = DERIVE_LEGACY;
			} else if (!strcmp("unified", string_option)) {
				opts->derive = DERIVE_UNIFIED;
			} else {
				kfree(string_option);
				goto invalid_option;
			}
			kfree(string_option);
			break;
		case Opt_lower_fs:
			string_option = match_strdup(&args[0]);
			if (!strcmp("ext4", string_option)) {
				opts->lower_fs = LOWER_FS_EXT4;
			} else if (!strcmp("fat", string_option)) {
				opts->lower_fs = LOWER_FS_FAT;
			} else {
				kfree(string_option);
				goto invalid_option;
			}
			kfree(string_option);
			break;
		case Opt_reserved_mb:
			if (match_int(&args[0], &option))
				return 0;
			opts->reserved_mb = option;
			break;
		/* unknown option */
		default:
invalid_option:
			if (!silent) {
				printk( KERN_ERR "Unrecognized mount option \"%s\" "
						"or missing value", p);
			}
			return -EINVAL;
		}
	}

	if (*debug) {
		printk( KERN_INFO "sdcardfs : options - debug:%d\n", *debug);
		printk( KERN_INFO "sdcardfs : options - uid:%d\n",
							opts->fs_low_uid);
		printk( KERN_INFO "sdcardfs : options - gid:%d\n",
							opts->fs_low_gid);
	}

	return 0;
}

/*
 * our custom d_alloc_root work-alike
 *
 * we can't use d_alloc_root if we want to use our own interpose function
 * unchanged, so we simply call our own "fake" d_alloc_root
 */
static struct dentry *sdcardfs_d_alloc_root(struct super_block *sb)
{
	struct dentry *ret = NULL;

	if (sb) {
		static const struct qstr name = {
			.name = "/",
			.len = 1
		};

		ret = d_alloc(NULL, &name);
		if (ret) {
			d_set_d_op(ret, &sdcardfs_ci_dops);
			ret->d_sb = sb;
			ret->d_parent = ret;
		}
	}
	return ret;
}

/*
 * There is no need to lock the sdcardfs_super_info's rwsem as there is no
 * way anyone can have a reference to the superblock at this point in time.
 */
static int sdcardfs_read_super(struct super_block *sb, const char *dev_name,
						void *raw_data, int silent)
{
	int err = 0;
	int debug;
	struct super_block *lower_sb;
	struct path lower_path;
	struct sdcardfs_sb_info *sb_info;
	void *pkgl_id;

	printk(KERN_INFO "sdcardfs version 2.0\n");

	if (!dev_name) {
		printk(KERN_ERR
		       "sdcardfs: read_super: missing dev_name argument\n");
		err = -EINVAL;
		goto out;
	}

	printk(KERN_INFO "sdcardfs: dev_name -> %s\n", dev_name);
	printk(KERN_INFO "sdcardfs: options -> %s\n", (char *)raw_data);

	/* parse lower path */
	err = kern_path(dev_name, LOOKUP_FOLLOW | LOOKUP_DIRECTORY,
			&lower_path);
	if (err) {
		printk(KERN_ERR	"sdcardfs: error accessing "
		       "lower directory '%s'\n", dev_name);
		goto out;
	}

	/* allocate superblock private data */
	sb->s_fs_info = kzalloc(sizeof(struct sdcardfs_sb_info), GFP_KERNEL);
	if (!SDCARDFS_SB(sb)) {
		printk(KERN_CRIT "sdcardfs: read_super: out of memory\n");
		err = -ENOMEM;
		goto out_free;
	}

	sb_info = sb->s_fs_info;

	/* parse options */
	err = parse_options(sb, raw_data, silent, &debug, &sb_info->options);
	if (err) {
		printk(KERN_ERR	"sdcardfs: invalid options\n");
		goto out_freesbi;
	}

	if (sb_info->options.derive != DERIVE_NONE) {
		pkgl_id = packagelist_create(sb_info->options.write_gid);
		if(IS_ERR(pkgl_id))
			goto out_freesbi;
		else
			sb_info->pkgl_id = pkgl_id;
	}

	/* set the lower superblock field of upper superblock */
	lower_sb = lower_path.dentry->d_sb;
	atomic_inc(&lower_sb->s_active);
	sdcardfs_set_lower_super(sb, lower_sb);

	/* inherit maxbytes from lower file system */
	sb->s_maxbytes = lower_sb->s_maxbytes;

	/*
	 * Our c/m/atime granularity is 1 ns because we may stack on file
	 * systems whose granularity is as good.
	 */
	sb->s_time_gran = 1;

	sb->s_magic = SDCARDFS_SUPER_MAGIC;
	sb->s_op = &sdcardfs_sops;

	/* see comment next to the definition of sdcardfs_d_alloc_root */
	sb->s_root = sdcardfs_d_alloc_root(sb);
	if (!sb->s_root) {
		err = -ENOMEM;
		goto out_sput;
	}

	/* link the upper and lower dentries */
	sb->s_root->d_fsdata = NULL;
	err = new_dentry_private_data(sb->s_root);
	if (err)
		goto out_freeroot;

	/* set the lower dentries for s_root */
	sdcardfs_set_lower_path(sb->s_root, &lower_path);

	/* call interpose to create the upper level inode */
	err = sdcardfs_interpose(sb->s_root, sb, &lower_path);
	if (!err) {
		/* setup permission policy */
		switch(sb_info->options.derive) {
			case DERIVE_NONE:
				setup_derived_state(sb->s_root->d_inode,
					PERM_ROOT, 0, AID_ROOT, AID_SDCARD_RW, 00775);
				sb_info->obbpath_s = NULL;
				break;
			case DERIVE_LEGACY:
				/* Legacy behavior used to support internal multiuser layout which
				 * places user_id at the top directory level, with the actual roots
				 * just below that. Shared OBB path is also at top level. */
				setup_derived_state(sb->s_root->d_inode,
				        PERM_LEGACY_PRE_ROOT, 0, AID_ROOT, AID_SDCARD_R, 00771);
				/* initialize the obbpath string and lookup the path
				 * sb_info->obb_path will be deactivated by path_put
				 * on sdcardfs_put_super */
				sb_info->obbpath_s = kzalloc(PATH_MAX, GFP_KERNEL);
				snprintf(sb_info->obbpath_s, PATH_MAX, "%s/obb", dev_name);
				err =  prepare_dir(sb_info->obbpath_s,
							sb_info->options.fs_low_uid,
							sb_info->options.fs_low_gid, 00755);
				if(err)
					printk(KERN_ERR "sdcardfs: %s: %d, error on creating %s\n",
							__func__,__LINE__, sb_info->obbpath_s);
				break;
			case DERIVE_UNIFIED:
				/* Unified multiuser layout which places secondary user_id under
				 * /Android/user and shared OBB path under /Android/obb. */
				setup_derived_state(sb->s_root->d_inode,
						PERM_ROOT, 0, AID_ROOT, AID_SDCARD_R, 00771);

				sb_info->obbpath_s = kzalloc(PATH_MAX, GFP_KERNEL);
				snprintf(sb_info->obbpath_s, PATH_MAX, "%s/Android/obb", dev_name);
				break;
		}
		fix_derived_permission(sb->s_root->d_inode);

		if (!silent)
			printk(KERN_INFO "sdcardfs: mounted on top of %s type %s\n",
						dev_name, lower_sb->s_type->name);
		goto out;
	}
	/* else error: fall through */

	free_dentry_private_data(sb->s_root);
out_freeroot:
	dput(sb->s_root);
out_sput:
	/* drop refs we took earlier */
	atomic_dec(&lower_sb->s_active);
	packagelist_destroy(sb_info->pkgl_id);
out_freesbi:
	kfree(SDCARDFS_SB(sb));
	sb->s_fs_info = NULL;
out_free:
	path_put(&lower_path);

out:
	return err;
}

/* A feature which supports mount_nodev() with options */
static struct dentry *mount_nodev_with_options(struct file_system_type *fs_type,
        int flags, const char *dev_name, void *data,
        int (*fill_super)(struct super_block *, const char *, void *, int))

{
	int error;
	struct super_block *s = sget(fs_type, NULL, set_anon_super, NULL);

	if (IS_ERR(s))
		return ERR_CAST(s);

	s->s_flags = flags;

	error = fill_super(s, dev_name, data, flags & MS_SILENT ? 1 : 0);
	if (error) {
		deactivate_locked_super(s);
		return ERR_PTR(error);
	}
	s->s_flags |= MS_ACTIVE;
	return dget(s->s_root);
}

struct dentry *sdcardfs_mount(struct file_system_type *fs_type, int flags,
			    const char *dev_name, void *raw_data)
{
	/*
	 * dev_name is a lower_path_name,
	 * raw_data is a option string.
	 */
	return mount_nodev_with_options(fs_type, flags, dev_name,
					raw_data, sdcardfs_read_super);
}

static struct file_system_type sdcardfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= SDCARDFS_NAME,
	.mount		= sdcardfs_mount,
	.kill_sb	= generic_shutdown_super,
	.fs_flags	= FS_REVAL_DOT,
};

static int __init init_sdcardfs_fs(void)
{
	int err;

	pr_info("Registering sdcardfs " SDCARDFS_VERSION "\n");

	err = sdcardfs_init_inode_cache();
	if (err)
		goto out;
	err = sdcardfs_init_dentry_cache();
	if (err)
		goto out;
	err = packagelist_init();
	if (err)
		goto out;
	err = register_filesystem(&sdcardfs_fs_type);
out:
	if (err) {
		sdcardfs_destroy_inode_cache();
		sdcardfs_destroy_dentry_cache();
		packagelist_exit();
	}
	return err;
}

static void __exit exit_sdcardfs_fs(void)
{
	sdcardfs_destroy_inode_cache();
	sdcardfs_destroy_dentry_cache();
	packagelist_exit();
	unregister_filesystem(&sdcardfs_fs_type);
	pr_info("Completed sdcardfs module unload\n");
}

MODULE_AUTHOR("Erez Zadok, Filesystems and Storage Lab, Stony Brook University"
	      " (http://www.fsl.cs.sunysb.edu/)");
MODULE_DESCRIPTION("Wrapfs " SDCARDFS_VERSION
		   " (http://wrapfs.filesystems.org/)");
MODULE_LICENSE("GPL");

module_init(init_sdcardfs_fs);
module_exit(exit_sdcardfs_fs);
