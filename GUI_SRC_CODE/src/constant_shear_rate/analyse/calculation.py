# pyright: reportAttributeAccessIssue=false

"""Numeric computation methods for AnalyseWindow, split out of the (formerly
huge) analyse_window_const_sr.py so the GUI/backend file stays a manageable
size.

AnalyseCalculationMixin is mixed into AnalyseWindow (see backend.py) via
multiple inheritance, so every method here runs with access to the same
self.data / self.time / self.offset_1 / ... instance attributes that
AnalyseWindow.__init__ sets up - it isn't a standalone class on its own.
"""
import numpy as np
from scipy.signal import savgol_filter


class AnalyseCalculationMixin:

    def data_calculation_function(self):
        if self.num_column == 5:

            self.determine_offsets_option()
            self.calculate_functions()
            self.calculate_all_mean()

            # remove last two columns
            self.final_data_to_show = self.final_data_to_save[:, :-4]

            self.take_Button.setDisabled(False)
            self.save_Button.setDisabled(False)

        elif self.num_column == 17:

            # take all the data up to column 13
            self.final_data_to_show = self.data[:, :-4]
            # take all the data 2 from the last column
            self.coefficient_saved = self.data[:, -4:]
            self.calculate_all_mean_after_save()
            self.reference_var_for_saved_data()

            self.take_Button.setDisabled(True)
            self.save_Button.setDisabled(True)

    def determine_offsets_option(self):

        # take from the main tab
        if self.take_Button.isChecked():

            self.offset_1 = float(self.worker_get_offset.data_4)
            self.offset_2 = float(self.worker_get_offset.data_6)
            self.textbox_offset1.setDisabled(True)
            self.textbox_offset2.setDisabled(True)
        else:
            # take from the textbox fields
            try:
                self.offset_1 = float(self.textbox_offset1.text())
                self.offset_2 = float(self.textbox_offset2.text())
                self.textbox_offset1.setDisabled(False)
                self.textbox_offset2.setDisabled(False)
            except ValueError:
                print("Invalid offset inputs")

    def calculate_functions(self):
        #############################################################################
        self.time = np.zeros((self.num_rows, 1))
        self.current_1 = np.zeros((self.num_rows, 1))
        self.current_2 = np.zeros((self.num_rows, 1))
        self.voltage_1 = np.zeros((self.num_rows, 1))
        self.voltage_2 = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field_unwrapped = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field_degree = np.zeros((self.num_rows, 1))
        self.angle_magnet = np.zeros((self.num_rows, 1))
        self.angle_magnet_unwrapped = np.zeros((self.num_rows, 1))
        self.angle_magnet_degree = np.zeros((self.num_rows, 1))
        self.phase_difference = np.zeros((self.num_rows, 1))
        self.angular_velocity = np.zeros((self.num_rows, 1))
        self.shear_rate = np.zeros((self.num_rows, 1))
        self.fr1on_moment = np.zeros((self.num_rows, 1))
        self.total_torque = np.zeros((self.num_rows, 1))
        self.magnitude_current = np.zeros((self.num_rows, 1))
        ###############################################################################

        # declare variables to read from the files (already given)
        self.time = self.data[:, 0]
        self.voltage_1 = self.data[:, 1]
        self.voltage_2 = self.data[:, 2]

        print("self.offset_1, self.offset_2",  self.offset_1)
        print(self.offset_2)
        self.data[:, 3] -= self.offset_1
        self.data[:, 4] -= self.offset_2

        self.current_1 = self.data[:, 3]
        self.current_2 = self.data[:, 4]

        self.label_fr.setText(f"f<sub>r0</sub> = {self.fr0}&nbsp;&nbsp;&nbsp;"
                              f"f<sub>r1</sub> = {self.fr1}&nbsp;&nbsp;&nbsp;")

        self.calculate_angle()
        self.calculate_magnitude_current()
        # calculate rotation velocity
        self.calculate_shear_rate()

        # calculate friction moment from shear rate
        self.calculate_friction_moment()
        self.calculate_shear_stress()
        self.calculate_viscosity()

        amf = self.angle_magnetic_field_degree.reshape(-1, 1)
        amag = self.angle_magnet_degree.reshape(-1, 1)
        pd = self.phase_difference_degree.reshape(-1, 1)
        angv = self.angular_velocity.reshape(-1, 1)
        trq = self.total_torque.reshape(-1, 1)
        sr1 = self.shear_rate.reshape(-1, 1)
        ss2 = self.shear_stress.reshape(-1, 1)
        vis = self.viscosity.reshape(-1, 1)

        # get the fr from packet tranmision
        self.fr0 = self.worker_get_fr_coefficient.fr0
        self.fr1 = self.worker_get_fr_coefficient.fr1

        # change text box for visibility
        self.textbox_offset1.setText(str(self.offset_1))
        self.textbox_offset2.setText(str(object=self.offset_2))

        N = self.angle_magnetic_field.shape[0]  # number of rows

        # make empty string columns
        self.off1_to_be_saved = np.full((N, 1), "", dtype=object)
        self.off2_to_be_saved = np.full((N, 1), "", dtype=object)
        self.fr0_to_be_saved = np.full((N, 1), "", dtype=object)
        self.fr1_to_be_saved = np.full((N, 1), "", dtype=object)

        # put user input only in the first row
        self.off1_to_be_saved[0, 0] = float(self.offset_1)
        self.off2_to_be_saved[0, 0] = float(self.offset_2)
        self.fr0_to_be_saved[0, 0] = float(self.fr0)
        self.fr1_to_be_saved[0, 0] = float(self.fr1)

        self.final_data_to_save = np.hstack((
            self.data,
            amf,
            amag,
            pd,
            angv,
            trq,
            sr1,
            ss2,
            vis,
            self.off1_to_be_saved,
            self.off2_to_be_saved,
            self.fr0_to_be_saved,
            self.fr1_to_be_saved
        ))

    def reference_var_for_saved_data(self):
        #############################################################################
        self.time = np.zeros((self.num_rows, 1))
        self.current_1 = np.zeros((self.num_rows, 1))
        self.current_2 = np.zeros((self.num_rows, 1))
        self.voltage_1 = np.zeros((self.num_rows, 1))
        self.voltage_2 = np.zeros((self.num_rows, 1))
        self.angle_magnetic_field = np.zeros((self.num_rows, 1))
        self.angle_magnet = np.zeros((self.num_rows, 1))
        self.phase_difference = np.zeros((self.num_rows, 1))
        self.angular_velocity = np.zeros((self.num_rows, 1))
        self.shear_rate = np.zeros((self.num_rows, 1))
        self.fr1on_moment = np.zeros((self.num_rows, 1))
        self.total_torque = np.zeros((self.num_rows, 1))
        self.magnitude_current = np.zeros((self.num_rows, 1))
        ###############################################################################

        # declare variables to read from the files (already given)
        self.time = self.final_data_to_show[:, 0]
        self.voltage_1 = self.final_data_to_show[:, 1]
        self.voltage_2 = self.final_data_to_show[:, 2]
        self.current_1 = self.final_data_to_show[:, 3]
        self.current_2 = self.final_data_to_show[:, 4]
        self.magnitude_current = np.hypot(self.current_1 , self.current_2)
        self.angle_magnetic_field = self.final_data_to_show[:, 5]
        self.angle_magnet = self.final_data_to_show[:, 6]
        self.phase_difference = self.final_data_to_show[:, 7]
        self.angular_velocity = self.final_data_to_show[:, 8]
        self.total_torque = self.final_data_to_show[:, 9]
        self.shear_rate = self.final_data_to_show[:, 10]
        self.shear_stress = self.final_data_to_show[:, 11]
        self.viscosity = self.final_data_to_show[:, 12]

        self.offset_1 = self.coefficient_saved[0, 0]
        self.offset_2 = self.coefficient_saved[0, 1]
        self.fr0 = self.coefficient_saved[0, 2]
        self.fr1 = self.coefficient_saved[0, 3]

        self.textbox_offset1.setText(str(self.offset_1))
        self.textbox_offset2.setText(str(object=self.offset_2))

    def calculate_angle(self):
        """
        Calculate magnet and magnetic field angles and their phase difference.

        This method computes the angles for the magnet and the magnetic field
        from the Hall voltage data stored in `self.data`.

        - The magnet angle is calculated using columns 2 and 3 (`self.data[:, 1]` and `self.data[:, 2]`).
        - The magnetic field angle is calculated using columns 4 and 5 (`self.data[:, 3]` and `self.data[:, 4]`).
        - Angles are unwrapped along axis 0 to remove discontinuities.
        - The phase difference between the magnetic field and the magnet is stored
        in `self.phase_difference`.

        :return: None
        """
        for row in range(self.num_rows):
            # angle from 2nd and 3rd columns (index 1 and 2)
            self.angle_magnet[row, 0] = np.arctan2(self.data[row, 2], self.data[row, 1])


            # angle from 4th and 5th columns (index 3 and 4)
            self.angle_magnetic_field[row, 0] = np.arctan2(self.data[row, 4], self.data[row, 3])
        # TODO: OFFSETS

        # calculate the angles

        self.angle_magnetic_field_degree = np.degrees(self.angle_magnetic_field) #[deg]
        self.angle_magnet_degree = np.degrees(self.angle_magnet) #[deg]

        self.angle_magnetic_field_unwrapped = np.unwrap(self.angle_magnetic_field, axis=0) #[rad]
        self.angle_magnet_unwrapped = np.unwrap(self.angle_magnet, axis=0) #[rad]

        self.phase_difference = self.angle_magnetic_field_unwrapped - self.angle_magnet_unwrapped # [rad]
        self.phase_difference_degree = np.degrees(self.phase_difference)  # [degree]

    def calculate_shear_rate(self):
        """
        Calculate the shear rate from the magnet angle signal.

        This method uses a Savitzky-Golay filter to smooth and differentiate
        the noisy angular position data (`self.angle_magnet`) with respect
        to time (`self.time`). The first derivative of the angle signal gives
        the angular velocity in [rad/s]. The shear rate is then obtained by
        scaling the angular velocity with the shear rate constant `C_SR`.

        Steps:
            1. Apply Savitzky-Golay filter to estimate angular velocity.
            2. Compute shear rate as angulalabelr_velocity * shear rate coefficient [C_SR].

        Updates Attributes:
            self.angular_velocity : np.ndarray
                Estimated angular velocity of the magnet [rad/s].
            self.shear_rate : np.ndarray
                Calculated shear rate [1/s].

        Notes:
            - The smoothness depends on the chosen `window_length` and `polyorder`.
            - `delta` is set as the mean time step from `self.time`.
        """
        self.angular_velocity = savgol_filter(
            self.angle_magnet_unwrapped[:, 0],
            window_length=101,  # try 51, 101
            polyorder=2,  # 2 or 3
            deriv=1,  # first derivative
            delta=np.mean(np.diff(self.time))  # time step
        )  # [rad /s]

        self.shear_rate = self.angular_velocity * self.C_SR  # [1 / s]

    def calculate_friction_moment(self):
        self.friction_moment = self.angular_velocity * self.fr1 + self.fr0  # - y_0        # [Nm]

    def calculate_magnitude_current(self):
        self.magnitude_current = np.hypot(self.current_1, self.current_2) # [mA]

    def calculate_shear_stress(self):

        self.total_torque = (
                self.CALIBRATION_FACTOR  # dimensionless
                * self.DIPOLE_MOMENT  # [A·m²]
                * self.COIL_CONSTANT  # [T/A]
                * (self.magnitude_current / 1000)  # mA → A
                * np.sin(self.phase_difference[:, 0])  # dimensionless
                - self.friction_moment  # [Nm]
        )
        self.shear_stress = self.total_torque * self.C_SS  # [Pa]

    def calculate_viscosity(self):
        self.viscosity = self.shear_stress / self.shear_rate  # [Pa * s]

    def calculate_all_mean(self):

        self.phase_difference_degree_mean = np.mean(self.phase_difference_degree)
        print(self.phase_difference_degree_mean)
        self.angular_velocity_mean = np.mean(self.angular_velocity)
        self.total_torque_mean = np.mean(self.total_torque)
        self.shear_rate_mean = np.mean(self.shear_rate)
        self.shear_stress_mean = np.mean(self.shear_stress)
        self.viscosity_mean = np.mean(self.viscosity)

    def calculate_all_mean_after_save(self):

        self.phase_difference_degree_mean = np.mean(self.data[:, 7])
        self.angular_velocity_mean = np.mean(self.data[:, 8])
        self.total_torque_mean = np.mean(self.data[:, 9])
        self.shear_rate_mean = np.mean(self.data[:, 10])
        self.shear_stress_mean = np.mean(self.data[:, 11])
        self.viscosity_mean = np.mean(self.data[:, 12])
