import h5py
import json
import numpy as np
import os
from typing import Iterable, Union
import cv2

class H5DataCollector:
    """Data collection interface for robomimic.

    This class implements a data collector interface for saving simulation states to disk.
    The data is stored in `HDF5`_ binary data format. The class is useful for collecting
    demonstrations. The collected data follows the `structure`_ from robomimic.

    All datasets in `robomimic` require the observations and next observations obtained
    from before and after the environment step. These are stored as a dictionary of
    observations in the keys "obs" and "next_obs" respectively.

    For certain agents in `robomimic`, the episode data should have the following
    additional keys: "actions", "rewards", "dones". This behavior can be altered by changing
    the dataset keys required in the training configuration for the respective learning agent.

    For reference on datasets, please check the robomimic `documentation`.

    .. _HDF5: https://www.h5py.org/
    .. _structure: https://robomimic.github.io/docs/datasets/overview.html#dataset-structure
    .. _documentation: https://github.com/ARISE-Initiative/robomimic/blob/master/robomimic/config/base_config.py#L167-L173
    """

    def __init__(
        self,
        env_name: str,
        directory_path: str,
        filename: str = "data",
        num_demos: int = 1,
        flush_freq: int = 1,
        env_config: dict = None,
    ):
        """Initializes the data collection wrapper.

        Args:
            env_name (str): The name of the environment.
            directory_path (str): The path to store collected data.
            filename (str, optional): The basename of the saved file. Defaults to "test".
            num_demos (int, optional): Number of demonstrations to record until stopping. Defaults to 1.
            flush_freq (int, optional): Frequency to dump data to disk. Defaults to 1.
            env_config (dict): The configuration for the environment. Defaults to None.
        """
        # save input arguments
        self._env_name = env_name
        self._env_config = env_config
        self._directory = os.path.abspath(directory_path)
        self._filename = filename
        self._num_demos = num_demos
        self._flush_freq = flush_freq
        # print info
        print(self.__str__())

        # create directory it doesn't exist
        if not os.path.isdir(self._directory):
            os.makedirs(self._directory)

        # placeholder for current hdf5 file object
        self._h5_file_stream = None
        self._h5_data_group = None
        self._h5_episode_group = None

        # store count of demos within episode
        self._demo_count = 0
        # flags for setting up
        self._is_first_interaction = True
        self._is_stop = False
        # create buffers to store data
        self._dataset = dict()

    def __del__(self):
        """Destructor for data collector."""
        if not self._is_stop:
            self.close()

    def __str__(self) -> str:
        """Represents the data collector as a string."""
        msg = "Dataset collector <class RobomimicDataCollector> object"
        msg += f"\tStoring trajectories in directory: {self._directory}\n"
        msg += f"\tNumber of demos for collection   : {self._num_demos}\n"
        msg += f"\tFrequency for saving data to disk: {self._flush_freq}\n"

        return msg

    """
    Properties
    """

    @property
    def demo_count(self) -> int:
        """The number of demos collected so far."""
        return self._demo_count

    """
    Operations.
    """

    def is_stopped(self) -> bool:
        """Whether data collection is stopped or not.

        Returns:
            bool: True if data collection has stopped.
        """
        return self._is_stop

    def reset(self):
        """Reset the internals of data logger."""
        # setup the file to store data in
        if self._is_first_interaction:
            self._demo_count = 0
            self._create_new_file(self._filename)
            self._is_first_interaction = False
        # clear out existing buffers
        self._dataset = dict()

    def add(self, key: str, value: np.ndarray):
        """Add a key-value pair to the dataset.

        The key can be nested by using the "/" character. For example:
        "obs/joint_pos". Currently only two-level nesting is supported.

        Args:
            key (str): The key name.
            value (np.ndarray): The corresponding value
                 of shape (N, ...), where `N` is number of environments.

        Raises:
            ValueError: When provided key has sub-keys more than 2. Example: "obs/joints/pos", instead
                of "obs/joint_pos".
        """
        # check if data should be recorded
        if self._is_first_interaction:
            print("Please call reset before adding new data. Calling reset...")
            self.reset()
        if self._is_stop:
            print(f"Desired number of demonstrations collected: {self._demo_count} >= {self._num_demos}.")
            return
        # check datatype
        value = np.asarray(value)
        # check if there are sub-keys
        sub_keys = key.split("/")
        num_sub_keys = len(sub_keys)
        if len(sub_keys) > 2:
            raise ValueError(f"Input key '{key}' has elements {num_sub_keys} which is more than two.")
        # add key to dictionary if it doesn't exist
        for i in range(value.shape[0]):
            # demo index
            if f"env_{i}" not in self._dataset:
                self._dataset[f"env_{i}"] = dict()
            # key index
            if num_sub_keys == 2:
                # create keys
                if sub_keys[0] not in self._dataset[f"env_{i}"]:
                    self._dataset[f"env_{i}"][sub_keys[0]] = dict()
                if sub_keys[1] not in self._dataset[f"env_{i}"][sub_keys[0]]:
                    self._dataset[f"env_{i}"][sub_keys[0]][sub_keys[1]] = list()
                # add data to key
                self._dataset[f"env_{i}"][sub_keys[0]][sub_keys[1]].append(value[i])
            else:
                # create keys
                if sub_keys[0] not in self._dataset[f"env_{i}"]:
                    self._dataset[f"env_{i}"][sub_keys[0]] = list()
                # add data to key
                self._dataset[f"env_{i}"][sub_keys[0]].append(value[i])

    def flush(self, env_ids: Iterable[int] = (0)):
        """Flush the episode data based on environment indices.

        Args:
            env_ids (Iterable[int], optional): Environment indices to write data for. Defaults to (0).
        """
        # check that data is being recorded
        if self._h5_file_stream is None or self._h5_data_group is None:
            print("No file stream has been opened. Please call reset before flushing data.")
            return
        # iterate over each environment and add their data
        for index in env_ids:
            # data corresponding to demo
            env_dataset = self._dataset[f"env_{index}"]
            # create episode group based on demo count
            h5_episode_group = self._h5_data_group.create_group(f"demo_{self._demo_count}")
            # store number of steps taken
            h5_episode_group.attrs["num_samples"] = len(env_dataset["actions"])
            # store other data from dictionary
            for key, value in env_dataset.items():
                if isinstance(value, dict):
                    # create group
                    key_group = h5_episode_group.create_group(key)
                    # add sub-keys values
                    for sub_key, sub_value in value.items():
                        key_group.create_dataset(sub_key, data=np.array(sub_value))
                else:
                    h5_episode_group.create_dataset(key, data=np.array(value))
            # increment total step counts
            self._h5_data_group.attrs["total"] += h5_episode_group.attrs["num_samples"]
            # increment total demo counts
            self._demo_count += 1
            # reset buffer for environment
            self._dataset[f"env_{index}"] = dict()
            # dump at desired frequency
            if self._demo_count % self._flush_freq == 0:
                self._h5_file_stream.flush()
                print(f">>> Flushing data to disk. Collected demos: {self._demo_count} / {self._num_demos}")
        # if demos collected then stop
        if self._demo_count >= self._num_demos:
            self.close()

    def close(self):
        """Stop recording and save the file at its current state."""
        if not self._is_stop:
            print(f">>> Closing recording of data. Collected demos: {self._demo_count} / {self._num_demos}")
            # close the file safely
            if self._h5_file_stream is not None:
                self._h5_file_stream.close()
            # mark that data collection is stopped
            self._is_stop = True

    """
    Helper functions.
    """

    def _create_new_file(self, fname: str):
        """Create a new HDF5 file for writing episode info into.

        Reference:
            https://robomimic.github.io/docs/datasets/overview.html

        Args:
            fname (str): The base name of the file.
        """
        if not fname.endswith(".hdf5"):
            fname += ".hdf5"
        # define path to file
        hdf5_path = os.path.join(self._directory, fname)
        # construct the stream object
        self._h5_file_stream = h5py.File(hdf5_path, "w")
        # create group to store data
        self._h5_data_group = self._h5_file_stream.create_group("data")
        # stores total number of samples accumulated across demonstrations
        self._h5_data_group.attrs["total"] = 0
        # store the environment meta-info
        # -- we use gym environment type
        # Ref: https://github.com/ARISE-Initiative/robomimic/blob/master/robomimic/envs/env_base.py#L15
        env_type = 2
        # -- check if env config provided
        if self._env_config is None:
            self._env_config = dict()
        # -- add info
        self._h5_data_group.attrs["env_args"] = json.dumps(
            {"env_name": self._env_name, "type": env_type, "env_kwargs": self._env_config}
        )
        
class DataCollector():
    def __init__(self, task, save_path) -> None:
        self.main_camera_image_hist = []
        self.wrist_camera_image_hist = []
        self.state_hist = []
        self.action_hist = []
        self.language_instruction_hist = []
        self.current_step = 0
        self.last_q = np.deg2rad([0.0, 0.0, 0.0, 0.0, -90.0, 0.0, 
                                  0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.save_path = save_path.replace('data/', f'data/{task}_') + '.npy'
        
    def add(self, image, q, dq, gripper, action):
        state = np.hstack([np.deg2rad(q), gripper, np.deg2rad(dq)])
        self.main_camera_image_hist.append(image)
        self.state_hist.append(state)
        self.action_hist.append(action)
    
    def save(self): 
        """ image_hist: list of np arrys of shape (img_s, img_s, 3) wrist_image_hist: list of np arrys of shape (img_s, img_s, 3) state_hist: list of np arrys of shape (n_dof,) action_hist: list of np arrys of shape (n_dof,) language_instruction_hist: list of strings """ 
        episode = []
        total_length = len(self.main_camera_image_hist)
        for step in range(total_length):
            image = cv2.imread(self.main_camera_image_hist[step])
            episode.append(
                {'image': np.asarray(image, dtype=np.uint8), 
                #  'wrist_image': np.asarray(wrist_camera_image_hist[step], dtype=np.uint8), 
                 'state': np.asarray(self.state_hist[step], dtype=np.float32), 
                 'action': np.asarray(self.action_hist[step], dtype=np.float32), 
                #  'language_instruction': language_instruction_hist[step], 
                }) 
        np.save(self.save_path, episode)
        