from __future__ import absolute_import, division, print_function

import rospy
from dynamic_reconfigure.client import Client as DynClient
from timeit import default_timer as timer

try:
    import typing
except ImportError:
    pass


class RealSenseCameraConfiguration:
    __timeout = 30

    def __init__(self, camera_name):
        self.__camera_name = camera_name

        # Connect reconfigure clients
        self.__rgb_module = DynClient(self.__camera_name + "/rgb_camera", timeout=self.__timeout)
        self.__depth_module = DynClient(self.__camera_name + "/stereo_module", timeout=self.__timeout)

    def set_rgb_config_callback(self, callback):
        self.__rgb_module.set_config_callback(callback)

    def set_depth_config_callback(self, callback):
        self.__depth_module.set_config_callback(callback)

    def update_rgb_parameter(self, name, value):
        self.__update_parameter(self.__rgb_module, name, value)

    def update_depth_parameter(self, name, value):
        self.__update_parameter(self.__depth_module, name, value)

    def get_rgb_parameter(self, name):
        return self.__get_parameter(self.__rgb_module, name)

    def get_depth_parameter(self, name):
        return self.__get_parameter(self.__depth_module, name)

    def get_rgb_parameter_info(self, name):
        return self.__get_parameter_info(self.__rgb_module, name)

    def get_depth_parameter_info(self, name):
        return self.__get_parameter_info(self.__depth_module, name)

    def __get_description(self, module, name, value=None):
        # type: (DynClient, str, typing.Optional[typing.Any]) -> typing.Union[dict, None]
        # Wait for module parameter_description and types to become available if not available
        if not module.param_description or not module._param_types:
            start_time = timer()
            while not module.param_description or not module._param_types:
                if timer() - start_time >= self.__timeout:
                    raise rospy.ROSException("Timeout reached waiting for parameter description")

        if name not in module._param_types:
            _available_parameters = ", ".join(module._param_types.keys())
            rospy.logerr("Parameter '{}' not in available parameters '{}'".format(name, _available_parameters))
            return None

        if value is not None:
            _param_type = module._param_types[name]
            if not isinstance(value, _param_type):
                rospy.logerr("Parameter '{}' is '{}' when it should be '{}'".format(name, type(value), _param_type))
                return None

        _param_descriptions = {d['name']: d for d in module.param_description}
        return _param_descriptions

    def __update_parameter(self, module, name, value):
        # type: (DynClient, str, typing.Any) -> None
        _param_descriptions = self.__get_description(module, name, value)
        if not _param_descriptions:
            return

        _min_v, _max_v = _param_descriptions[name]['min'], _param_descriptions[name]['max']
        if not isinstance(module._param_types[name], bool) and _max_v >= value <= _min_v:
            rospy.logerr("Cannot set parameter to '{}' valid range is '{}'-'{}'".format(value, _min_v, _max_v))
            return

        module.update_configuration({name: value})

    def __get_parameter(self, module, name):
        # type: (DynClient, str) -> typing.Union[typing.Any, None]
        _param_descriptions = self.__get_description(module, name)
        if not _param_descriptions:
            return None

        if not hasattr(module.config, name):
            start_time = timer()
            while not hasattr(module.config, name):
                if timer() - start_time >= self.__timeout:
                    rospy.logerr("Parameter '{}' does not exist in current config".format(name))
                    return None

        return getattr(module.config, name)

    def __get_parameter_info(self, module, name):
        # type: (DynClient, str) -> typing.Union[dict, None]
        _param_descriptions = self.__get_description(module, name)
        if not _param_descriptions:
            return None
        return _param_descriptions[name]

    def set_laser_power(self, value):
        self.update_depth_parameter('laser_power', value)

    def get_laser_power(self):
        return self.get_depth_parameter('laser_power')

    def get_laser_power_info(self):
        return self.get_depth_parameter_info('laser_power')

    def set_output_trigger_enabled(self, value):
        self.update_depth_parameter('output_trigger_enabled', value)

    def get_output_trigger_enabled(self):
        return self.get_depth_parameter('output_trigger_enabled')

    def get_output_trigger_enabled_info(self):
        return self.get_depth_parameter_info('output_trigger_enabled')

    def set_emitter_enabled(self, value):
        self.update_depth_parameter('emitter_enabled', value)

    def get_emitter_enabled(self):
        return self.get_depth_parameter('emitter_enabled')

    def get_emitter_enabled_info(self):
        return self.get_depth_parameter_info('emitter_enabled')

    def set_enable_depth_auto_exposure(self, value):
        self.update_depth_parameter('enable_auto_exposure', value)

    def get_enable_depth_auto_exposure(self):
        return self.get_depth_parameter('enable_auto_exposure')

    def get_enable_depth_auto_exposure_info(self):
        return self.get_depth_parameter_info('enable_auto_exposure')

    def set_global_depth_time_enabled(self, value):
        self.update_depth_parameter('global_time_enabled', value)

    def get_global_depth_time_enabled(self):
        return self.get_depth_parameter('global_time_enabled')

    def get_global_depth_time_enabled_info(self):
        return self.get_depth_parameter_info('global_time_enabled')

    def set_inter_cam_sync_mode(self, value):
        self.update_depth_parameter('inter_cam_sync_mode', value)

    def get_inter_cam_sync_mode(self):
        return self.get_depth_parameter('inter_cam_sync_mode')

    def get_inter_cam_sync_mode_info(self):
        return self.get_depth_parameter_info('inter_cam_sync_mode')

    def set_visual_preset(self, value):
        self.update_depth_parameter('visual_preset', value)

    def get_visual_preset(self):
        return self.get_depth_parameter('visual_preset')

    def get_visual_preset_info(self):
        return self.get_depth_parameter_info('visual_preset')

    def set_error_polling_enabled(self, value):
        self.update_depth_parameter('error_polling_enabled', value)

    def get_error_polling_enabled(self):
        return self.get_depth_parameter('error_polling_enabled')

    def get_error_polling_enabled_info(self):
        return self.get_depth_parameter_info('error_polling_enabled')

    def set_depth_gain(self, value):
        self.update_depth_parameter('gain', value)

    def get_depth_gain(self):
        return self.get_depth_parameter('gain')

    def get_depth_gain_info(self):
        return self.get_depth_parameter_info('gain')

    def set_depth_frames_queue_size(self, value):
        self.update_depth_parameter('frames_queue_size', value)

    def get_depth_frames_queue_size(self):
        return self.get_depth_parameter('frames_queue_size')

    def get_depth_frames_queue_size_info(self):
        return self.get_depth_parameter_info('frames_queue_size')

    def set_emitter_on_off(self, value):
        self.update_depth_parameter('emitter_on_off', value)

    def get_emitter_on_off(self):
        return self.get_depth_parameter('emitter_on_off')

    def get_emitter_on_off_info(self):
        return self.get_depth_parameter_info('emitter_on_off')

    def set_depth_exposure(self, value):
        self.update_depth_parameter('exposure', value)

    def get_depth_exposure(self):
        return self.get_depth_parameter('exposure')

    def get_depth_exposure_info(self):
        return self.get_depth_parameter_info('exposure')

    def set_gamma(self, value):
        self.update_rgb_parameter('gamma', value)

    def get_gamma(self):
        return self.get_rgb_parameter('gamma')

    def get_gamma_info(self):
        return self.get_rgb_parameter_info('gamma')

    def set_saturation(self, value):
        self.update_rgb_parameter('saturation', value)

    def get_saturation(self):
        return self.get_rgb_parameter('saturation')

    def get_saturation_info(self):
        return self.get_rgb_parameter_info('saturation')

    def set_brightness(self, value):
        self.update_rgb_parameter('brightness', value)

    def get_brightness(self):
        return self.get_rgb_parameter('brightness')

    def get_brightness_info(self):
        return self.get_rgb_parameter_info('brightness')

    def set_enable_rgb_auto_exposure(self, value):
        self.update_rgb_parameter('enable_auto_exposure', value)

    def get_enable_rgb_auto_exposure(self):
        return self.get_rgb_parameter('enable_auto_exposure')

    def get_enable_rgb_auto_exposure_info(self):
        return self.get_rgb_parameter_info('enable_auto_exposure')

    def set_sharpness(self, value):
        self.update_rgb_parameter('sharpness', value)

    def get_sharpness(self):
        return self.get_rgb_parameter('sharpness')

    def get_sharpness_info(self):
        return self.get_rgb_parameter_info('sharpness')

    def set_frames_queue_size(self, value):
        self.update_rgb_parameter('frames_queue_size', value)

    def get_frames_queue_size(self):
        return self.get_rgb_parameter('frames_queue_size')

    def get_frames_queue_size_info(self):
        return self.get_rgb_parameter_info('frames_queue_size')

    def set_global_rgb_time_enabled(self, value):
        self.update_rgb_parameter('global_time_enabled', value)

    def get_global_rgb_time_enabled(self):
        return self.get_rgb_parameter('global_time_enabled')

    def get_global_rgb_time_enabled_info(self):
        return self.get_rgb_parameter_info('global_time_enabled')

    def set_hue(self, value):
        self.update_rgb_parameter('hue', value)

    def get_hue(self):
        return self.get_rgb_parameter('hue')

    def get_hue_info(self):
        return self.get_rgb_parameter_info('hue')

    def set_white_balance(self, value):
        self.update_rgb_parameter('white_balance', value)

    def get_white_balance(self):
        return self.get_rgb_parameter('white_balance')

    def get_white_balance_info(self):
        return self.get_rgb_parameter_info('white_balance')

    def set_power_line_frequency(self, value):
        self.update_rgb_parameter('power_line_frequency', value)

    def get_power_line_frequency(self):
        return self.get_rgb_parameter('power_line_frequency')

    def get_power_line_frequency_info(self):
        return self.get_rgb_parameter_info('power_line_frequency')

    def set_backlight_compensation(self, value):
        self.update_rgb_parameter('backlight_compensation', value)

    def get_backlight_compensation(self):
        return self.get_rgb_parameter('backlight_compensation')

    def get_backlight_compensation_info(self):
        return self.get_rgb_parameter_info('backlight_compensation')

    def set_rgb_gain(self, value):
        self.update_rgb_parameter('gain', value)

    def get_rgb_gain(self):
        return self.get_rgb_parameter('gain')

    def get_rgb_gain_info(self):
        return self.get_rgb_parameter_info('gain')

    def set_rgb_auto_exposure_priority(self, value):
        self.update_rgb_parameter('auto_exposure_priority', value)

    def get_rgb_auto_exposure_priority(self):
        return self.get_rgb_parameter('auto_exposure_priority')

    def get_rgb_auto_exposure_priority_info(self):
        return self.get_rgb_parameter_info('auto_exposure_priority')

    def set_rgb_exposure(self, value):
        self.update_rgb_parameter('exposure', value)

    def get_rgb_exposure(self):
        return self.get_rgb_parameter('exposure')

    def get_rgb_exposure_info(self):
        return self.get_rgb_parameter_info('exposure')

    def set_rgb_contrast(self, value):
        self.update_rgb_parameter('contrast', value)

    def get_rgb_contrast(self):
        return self.get_rgb_parameter('contrast')

    def get_rgb_contrast_info(self):
        return self.get_rgb_parameter_info('contrast')

    def set_enable_rgb_auto_white_balance(self, value):
        self.update_rgb_parameter('enable_auto_white_balance', value)

    def get_enable_rgb_auto_white_balance(self):
        return self.get_rgb_parameter('enable_auto_white_balance')

    def get_enable_rgb_auto_white_balance_info(self):
        return self.get_rgb_parameter_info('enable_auto_white_balance')
