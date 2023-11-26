import serial
import serial.tools.list_ports
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt

class nanovna_v2:
    ###############################################################
    #                    Constructor/Destructor                   #
    ###############################################################
    def __init__(self, port, baud = 115200, timeout = 5):
        self.port_name = port
        self.baud = baud
        self.timeout = timeout

        self.sweepStartHz = None
        self.sweepStopHz = None
        self.points = None

        self._raw_sweep_data = None

        self.ser = serial.Serial(self.port_name, baudrate=self.baud, timeout=self.timeout)

        # Get Version information
        self.device_variant = self._bytes_to_int(self._read_reg(self.addr.deviceVariant, 1))
        self.protocol_version = self._bytes_to_int(self._read_reg(self.addr.protocolVersion, 1))
        self.hardware_revision = self._bytes_to_int(self._read_reg(self.addr.hardwareRevision, 1))
        self.firmware_major = self._bytes_to_int(self._read_reg(self.addr.firmwareMajor, 1))
        self.firmware_minor = self._bytes_to_int(self._read_reg(self.addr.firmwareMinor, 1))

        self._close()        
    
    def __del__(self):
        self._close()
    
    def _open(self):
        if not self.ser.is_open:
            self.ser.open()

    def _close(self):
        if self.ser.is_open:
            self.ser.close()



    ###############################################################
    #                         VNA Commands                        #
    ###############################################################
    
    def get_device_variant(self) -> str:
        return f"{self.device_variant}"
    
    def get_protocol_version(self) -> str:
        return f"{self.protocol_version}"
    
    def get_hardware_revision(self) -> str:
        return f"{self.hardware_revision}"
    
    def get_firmware_version(self) -> str:
        return f"{self.firmware_major}.{self.firmware_minor}"
    
    def get_device_information(self) -> str:
        dev_info = (
            f"NanoVNA v2 Information\r\n"
            f"\tDevice variant: {self.device_variant}\r\n"
            f"\tProtocol version: {self.protocol_version}\r\n"
            f"\tHardware revision: {self.hardware_revision}\r\n"
            f"\tFirmware version: {self.firmware_major}.{self.firmware_minor}"
        )

        return dev_info
    
    def set_sweep(self, start, stop, points = 101):
        if start is not None:
            self.sweepStartHz = start
        if stop is not None:
            self.sweepStopHz = stop
        if points is not None:
            self.points = points
        self._updateSweep()

    def _updateSweep(self):
        sweepStepHz = 0.
        if self.points > 1:
            sweepStepHz = (self.sweepStopHz - self.sweepStartHz) / (self.points - 1)

        self._write_reg(self.addr.sweepStartHz, self._int_to_bytes(self.sweepStartHz, 8))
        self._write_reg(self.addr.sweepStepHz, self._int_to_bytes(sweepStepHz, 8))
        self._write_reg(self.addr.sweepPoints, self._int_to_bytes(self.points, 2))
    
    def get_s11_s21(self) -> tuple[np.ndarray, np.ndarray]:
        # Get sweep data
        self._read_values_fifo()

        # Return empty lists if there is no valid data
        if self._raw_sweep_data is None:
            return (list(), list(), list())
        
        # Construct S-Params
        arr = np.array(self._raw_sweep_data).T
        
        fwd0_raw = arr[0]
        refl_raw = arr[1]
        thru_raw = arr[2]

        s11 = refl_raw / fwd0_raw
        s21 = thru_raw / fwd0_raw

        return (s11, s21)


    



    ###############################################################
    #                     Interface functions                     #
    ###############################################################
    def _bytes_to_int(self, val: bytes, signed: bool = False) -> int:
        return int.from_bytes(val, byteorder='little', signed=signed)
    
    def _int_to_bytes(self, val: bytes, len: int, signed : bool = False) -> bytes:
        return int.to_bytes(int(val), len, 'little', signed=signed)


    def _read_reg(self, address: bytes, num_bytes: int) -> bytes:
        if num_bytes not in (1, 2, 4):
            raise Exception(f"Cannot read {num_bytes}-byte register from nanoVNA v2. Acceptable values are 1, 2, and 4.")

        # Attempt to read byte from address
        self._open()
        self.ser.reset_input_buffer()

        msg = self.commands[f'READ{num_bytes}'] + address
        self.ser.write(msg)

        recv = self.ser.read(num_bytes)

        self._close()
        
        # Error if invalid data received
        if len(recv) != num_bytes:
            return "INVALID"
        
        # Else return data
        return recv

    def _write_reg(self, address: bytes, data: bytes):
        num_bytes = len(data)

        if num_bytes not in (1, 2, 4, 8):
            msg = f"Cannot write {num_bytes} bytes to register on nanoVNA v2. Acceptable lengths are 1, 2, 4, and 8."
            raise Exception(msg)

        # Write bytes to register
        self._open()

        msg = self.commands[f'WRITE{num_bytes}'] + address + data
        self.ser.write(msg)

        self._close()

    def _read_values_fifo(self) -> None:
        # First, trigger new FIFO reading
        self._write_reg(self.addr.valuesFIFO, b'\x00')

        # Request read from FIFO
        if self.points is None:
            raise RuntimeError("Number of points is not set. Try calling set_sweep() before calling this command.")

        # Acquire points   
        self._open()

        num_bytes = self.points * 32

        self.ser.write(self.commands.READFIFO + self.addr.valuesFIFO + self._int_to_bytes(self.points, 2)) # Trigger FIFO read
        arr = self.ser.read(num_bytes) # Get FIFO readings

        self._close()

        # Verify data was received correctly
        if len(arr) != num_bytes:
            return []
        
        # Process raw FIFO values to usable data
        self._raw_sweep_data = [()]*self.points

        for i in range(self.points):
            offset = i * 32
            data = arr[offset : offset + 32]

            # Parse Data
            fwd0 = complex(self._bytes_to_int(data[0:4], True), self._bytes_to_int(data[4:8], True))
            refl = complex(self._bytes_to_int(data[8:12], True), self._bytes_to_int(data[12:16], True))
            thru = complex(self._bytes_to_int(data[16:20], True), self._bytes_to_int(data[20:24], True))

            # Add raw values to raw sweep array
            self._raw_sweep_data[i] = (fwd0, refl, thru)

    
    ###############################################################
    #                          CONSTANTS                          #
    ###############################################################
    class commands(bytes, Enum):
        NOP = b'\x00'
        INDICATE = b'\x0d'
        READ1 = b'\x10'
        READ2 = b'\x11'
        READ4 = b'\x12'
        READFIFO = b'\x18'
        WRITE1 = b'\x20'
        WRITE2 = b'\x21'
        WRITE4 = b'\x22'
        WRITE8 = b'\x23'
        WRITEFIFO = b'\x28'

    class addr(bytes, Enum):
        sweepStartHz = b'\x00'
        sweepStepHz = b'\x10'
        sweepPoints = b'\x20'
        valuesPerFrequency = b'\x22'
        rawSamplesMode = b'\x26'
        valuesFIFO = b'\x30'
        deviceVariant = b'\xf0'
        protocolVersion = b'\xf1'
        hardwareRevision = b'\xf2'
        firmwareMajor = b'\xf3'
        firmwareMinor = b'\xf4'



def main():
    # Show available serial ports
    available_ports = serial.tools.list_ports.comports()

    print("Available Ports:")
    for idx, port in enumerate(available_ports):
        print(f"{idx:2d}: {port}")

    # Get Serial Port
    port_idx = int(input("Index of Serial Port> "))

    # Create VNA instance
    print(f"Opening nanoVNA v2 on {available_ports[port_idx]}")
    vna = nanovna_v2(available_ports[port_idx].device)

    # Show device information 
    print(vna.get_device_information())

    # Get S-Params
    vna.set_sweep(1E6, 101E6, 201)
    s11, s21 = vna.get_s11_s21()

    # Create mag/phase plot for S11 and S21
    fig, axs = plt.subplots(2, 1)

    freqs = np.linspace(vna.sweepStartHz, vna.sweepStopHz, vna.points)

    mag_phase_plot(axs[0], freqs, s11)
    mag_phase_plot(axs[1], freqs, s21)

    plt.show()

def mag_phase_plot(ax: plt.axes, freqs: np.ndarray, s: np.ndarray):
    # Put S-Params in log-mag form
    s_mag = 20*np.log10(np.abs(s))
    s_phase = np.angle(s)*180/np.pi

    # Create dual y-axis
    mag_plot = ax
    phase_plot = ax.twinx()
    phase_plot.set_ylim([-180, 180])

    # Plot mag & phase
    s_mag_ln = mag_plot.plot(freqs, s_mag, 'b', label="Magnitude")
    s_phase_ln = phase_plot.plot(freqs, s_phase, 'r', label="Phase")
    
    # Labels
    mag_plot.set_xlabel("Frequency (Hz)")
    mag_plot.set_ylabel("Magnitude (dB)")
    phase_plot.set_ylabel("Phase (deg)")

    # Legend
    lns = s_mag_ln + s_phase_ln
    labs = [l.get_label() for l in lns]
    mag_plot.legend(lns, labs, loc='best')


if __name__ == '__main__':
    main()