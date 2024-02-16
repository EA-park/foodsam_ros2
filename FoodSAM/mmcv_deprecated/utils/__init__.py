# flake8: noqa
# Copyright (c) Open-MMLab. All rights reserved.
from .config import DictAction, Config, ConfigDict
from .misc import deprecated_api_warning, is_list_of, is_tuple_of, is_seq_of
from .path import (check_file_exist, fopen, is_filepath, mkdir_or_exist,
                   scandir, symlink)
from .registry import Registry, build_from_cfg
from .version_utils import get_git_hash
from .logging import get_logger, print_log

try:
    import torch
except ImportError:
    __all__ = [
        'Config', 'ConfigDict', 'DictAction',
        'deprecated_api_warning', 'is_list_of', 'is_tuple_of', 'is_seq_of',
        'check_file_exist', 'fopen', 'is_filepath', 'mkdir_or_exist', 
        'scandir', 'symlink', 'build_from_cfg',
        'get_git_hash', 'get_logger', 'print_log'
    ]
else:
    from .parrots_wrapper import TORCH_VERSION
    from .registry import Registry
    __all__ = [
        'Config', 'ConfigDict', 'DictAction', 
        'deprecated_api_warning', 'is_list_of', 'is_tuple_of', 'is_seq_of',
        'check_file_exist', 'fopen', 'is_filepath', 'mkdir_or_exist', 
        'scandir', 'symlink',
        'get_git_hash', 'Registry', 'TORCH_VERSION', 
        'build_from_cfg', 'get_logger', 'print_log'
    ]
