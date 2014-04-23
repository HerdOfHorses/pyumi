#/usr/bin/env python2

import sys
import serial
import time

IPDONTCARE                      = -1
IP0                             =  0
IP1                             =  1

CMD_MASK                        = 0xc0
CMD_SV                          = 0x00
CMD_PA                          = 0x40
CMD_DI                          = 0x80
CMD_IP                          = 0xc0

""" SV: Simple commands """
CMD_GO                          = 0x00
CMD_IDENTIFY                    = 0x01
CMD_INIT                        = 0x08
CMD_START_SOAK                  = 0x09
CMD_INIT_START_SOAK             = 0x0a
CMD_STOP_SOAK                   = 0x0b

""" SV: Status """
CMD_STAT_BASE                   = 0x10
CMD_STAT_CTL_0                  = 0x10
CMD_STAT_CTL_1                  = 0x11
CMD_STAT_CTL_2                  = 0x12
CMD_STAT_CTL_3                  = 0x13
CMD_STAT_CTL_4                  = 0x14

GENERAL                         = 0x07
CMD_STAT_GENERAL                = 0x17

""" Mode settings: parameter = controller number. """
CMD_MODE_SET                    = 0x18

CMD_MODE_SET_POS                = 0x18
CMD_MODE_SET_FRC                = 0x19
CMD_MODE_SET_ABS                = 0x1a
CMD_MODE_SET_REL                = 0x1b
CMD_MODE_SET_UIOI               = 0x1c
CMD_MODE_SET_UIOO               = 0x1d

""" IP control """
CMD_IP_RESET                    = 0x20
CMD_IP_DEF_HOME                 = 0x21
CMD_IP_DEFAULT                  = 0x22

CMD_XXXX_STOP                   = 0x24
CMD_DEAD_STOP                   = 0x24
CMD_RAMP_STOP                   = 0x25
CMD_FREE_STOP                   = 0x26
CMD_FREE_OFF                    = 0x27

CMD_TOGGLE_OFF                  = 0x28
CMD_TOGGLE_ONCE                 = 0x29
CMD_TOGGLE_ON                   = 0x2a

""" CMD_SET_BAUD                = 0x30  """

CMD_VERSION                     = 0x38

"""
 * Parameter commands
 """

CMD_IM_READ_ERR                 = 0x40  # + ctl
CMD_IM_READ_CP                  = 0x48  # + ctl

CMD_IM_WRITE_CP                 = 0x58  # + ctl

CMD_DEF_READ1                   = 0x60  # Param = #of ctl param
CMD_DEF_READ2                   = 0x68  # + ctl

CMD_DEF_WRITE1                  = 0x70  # Param = #of ctl param
CMD_DEF_WRITE2                  = 0x78  # + ctl
"""
 * Directive commands
 """

CMD_DIR_MAN                     = 0x80

CMD_DIR_NUM                     = 0xa0
"""
 * Interpolation commands.
 """

CMD_INTERP                      = 0xe0

"""
 * SV responses (1 byte)
 """

RESP_TYPE_MASK                  = 0xf0
RESP_TYPE_SV_SIMPLE             = 0x00
RESP_TYPE_SV_STAT               = 0x10
RESP_TYPE_SV_ID                 = 0x20
RESP_TYPE_IM_READ               = 0x80
RESP_TYPE_IM_WRITE              = 0xa0
RESP_TYPE_DEF_R1                = 0xc0
RESP_TYPE_DEF_R2                = 0xd0
RESP_TYPE_DEF_W1                = 0xe0
RESP_TYPE_DEF_W2                = 0xf0

RESP_ACK                        = 0x00
RESP_PROGRESS                   = 0x01
RESP_STORED                     = 0x02
RESP_AXIS_BUSY                  = 0x03
RESP_IP_BUSY                    = 0x04
RESP_PARAM_OOR                  = 0x05
RESP_READ_ONLY                  = 0x06
RESP_READ_ONLY1                 = 0x07
RESP_SELECT_OOR                 = 0x08
RESP_CMD_OOR                    = 0x09
RESP_CMD_NOT_SUP                = 0x0a
RESP_FRAME_TIMEO                = 0x0b
RESP_FRAME_OVERRUN              = 0x0c
RESP_PARITY_ERROR               = 0x0d
RESP_IP_RESTART1                = 0x0e
RESP_IP_RESTART                 = 0x0f

"""
 * Status (3 bytes)
 * Byte one = status
 * Byte 2 = IP
 """
RESP_STAT_BASE                  = 0x10
RESP_STAT_CTL_0                 = 0x10
RESP_STAT_CTL_1                 = 0x11
RESP_STAT_CTL_2                 = 0x12
RESP_STAT_CTL_3                 = 0x13
RESP_STAT_CTL_4                 = 0x14

RESP_STAT_GENERAL               = 0x17

RESP_MASK_TASKS                 = 0x01
RESP_MASK_AXIS                  = 0x02
RESP_MASK_ERR_LIMITS            = 0x04
RESP_MASK_AXIS_RESET            = 0x08
RESP_MASK_USER_IO               = 0x10
RESP_MASK_MOTOR_REL             = 0x10
RESP_MASK_USER_IO_CONF          = 0x20
RESP_MASK_MOTOR_FRC             = 0x20

RESP_IDENTIFY_MASK              = 0x07
RESP_IDENTIFY_0                 = 0x20
RESP_IDENTIFY_1                 = 0x21

RESP_RW_MASK                    = 0xf0
RESP_IM_READ                    = 0x80  #""" + cksum """
RESP_IM_WRITE                   = 0xa0  #""" + cksum """

RESP_DEF_READ1                  = 0xc0
RESP_DEF_READ2                  = 0xd0  #""" + cksum """
RESP_DEF_WRITE1                 = 0xe0
RESP_DEF_WRITE2                 = 0xf0  #""" + cksum """

current_millis = lambda: int(round(time.time() * 1000))

def umi_cmd(cmd):
    a = bytearray()
    for c in cmd:
        a.append(c)
    return a

class UMI(object):
    def __init__(self, port):
        self.port = port

        s = serial.Serial()
        s.port = port
        s.baudrate = 9600
        s.timeout = 0.1
        s.writeTimeout = 5

        self.serial = s
        self.current_ip = 0
        self.toggle_mode = 0

        self.initialised = False
        self.initialising = False
        self.svsim = 0
        self.erresp = 0
        self.lasterr = 0
        self.errcmd = 0
        self.ncmds = 0
        self.resync = 0

        self.lasttime = 0

    def open(self):
        sys.stdout.write('UMI: opening port: %s\n' % (self.serial.port))
        self.serial.open()

    def raw(self, ip, cmd):
        ioerr = 0

        if not self.serial.isOpen():
            return ()

        if not (self.initialising ^ self.initialised):
            sys.stderr.write('Communications not initialised\n')
            #return None

        if ip != IPDONTCARE and ip != self.current_ip:
            if (toggle_mode):
                resp = self.raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
                if len(resp) == 0:
                    return ()
                if resp[0] != (RESP_IDENTIFY_0 + (1 - self.current_ip)):
                    if not rtx_resync(1):
                        return ()
            else:
                self.current_ip = 1 - self.current_ip
                if not self.raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ONCE))):
                    return ()

        time.sleep((self.lasttime + 8 - current_millis()) / 1000.0)
        self.lasttime = current_millis()
        sys.stdout.write('UMI writing: %s\n' % (repr(cmd)))
        self.serial.write(cmd)

        if not self.initialising and self.toggle_mode == True:
            self.current_ip = 1 - self.current_ip

        self.serial.timeout = (self.lasttime + 8 - current_millis()) / 1000.0
        resp = self.serial.read(3)
        sys.stdout.write('read: %s\n' % (repr(resp)))

        if len(resp) == 0:
            sys.stderr.write('no response\n')
            self.resync(1)
            return ()

        if len(resp) > 3:
            self.resync(1)
            sys.stderr.write('response overrun\n')
            return ()

        sys.stdout.write('checking response\n')
        expect = 1

        resp_type = ord(resp[0]) & RESP_TYPE_MASK
        if resp_type == RESP_TYPE_SV_ID:
            self.nid = self.nid + 1
            if cmd[0] == CMD_VERSION:
                ioerr = len(resp) != 3
            else:
                ioerr = len(resp) != 1
        elif resp_type == RESP_TYPE_SV_SIMPLE:
            self.svsim = self.svsim + 1
            if resp[0] >= 0x01 and resp[0] <= 0x0f:
                self.erresp = self.erresp + 1
                self.lasterr = resp[0]
                self.errcmd = (ip, cmd)
            self.ioerr = len(resp) != 1
        elif ((resp_type == RESP_TYPE_IM_WRITE) or
             (resp_type == RESP_TYPE_DEF_W1)   or
             (resp_type == RESP_TYPE_DEF_W2)):
            ioerr = len(resp) != 1
        elif resp_type == RESP_TYPE_DEF_R1:
            self.nreads = self.nreads + 1
            ioerr = len(resp) != 1
        elif ((resp_type == RESP_TYPE_SV_STAT) or
             (resp_type == RESP_TYPE_IM_READ) or
             (resp_type == RESP_TYPE_DEF_R2)):
            self.nreads = self.nreads + 1
            ioerr = len(resp) != 3
            expect = 3
        else:
            self.rtxerr = RESPONSE_UNKNOWN
            return ()

        if ioerr > 0:
            self.resync(1)
            if expect > len(resp):
                self.rtxerr = RESPONSE_INCOMPLETE
            elif expect < len(resp):
                self.rtxerr = RESPONSE_OVERRUN
            else:
                self.rtxerr = RESPONSE_UNKNOWN
            return ()

        sys.stdout.write('UMI: command completed\n')
        self.ncmds = self.ncmds + 1

        return resp

    def toggle_mode(self, mode):
        ip1 = 0
        ip2 = 0
        old = self.toggle_mode

        if mode == self.toggle_mode:
            return old

        self.rtxerr = COMMS_FAULT

        if self.toggle.mode:
            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_OFF)))
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                return -1

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ONCE)))
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                return -1

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_OFF)))
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                return -1

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
            if ((len(resp) != 1) or
               (ord(resp[0]) != RESP_IDENTIFY_0) and
               (ord(resp[0]) != RESP_IDENTIFY_1)):
                return -1

            ip1 = ord(resp[0]) & RESP_IDENTIFY_MASK

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ONCE)))
            if len(resp) == 0:
                return -1

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
            if ((len(resp) != 1) or
               (ord(resp[0]) != RESP_IDENTIFY_0) and
               (ord(resp[0]) != RESP_IDENTIFY_1)):
                return -1

            ip2 = ord(resp[0]) & RESP_IDENTIFY_MASK

            if ip1 == ip2:
                return -1

            self.current_ip = ip2

        else:
            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ON)))
            if len(resp) == 0:
                return -1

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ON)))
            if len(resp) == 0:
                return -1

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
            if ((len(resp) != 1) or
               (ord(resp[0]) != RESP_IDENTIFY_0) and
               (ord(resp[0]) != RESP_IDENTIFY_1)):
                return -1

            ip1 = ord(resp[0]) & RESP_IDENTIFY_MASK

            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
            if ((len(resp) != 1) or
               (ord(resp[0]) != RESP_IDENTIFY_0) and
               (ord(resp[0]) != RESP_IDENTIFY_1)):
                return -1

            ip2 = ord(resp[0]) & RESP_IDENTIFY_MASK

            if ip1 == ip2:
                return -1

            self.current_ip = ip1

        self.toggle_mode = m
        return old


    def resync(self, err):
        ip = self.current_ip
        m = self.toggle_mode

        sys.stdout.write("UMI: resync\n")

        if not hasattr(self.resync, "lock"):
            self.resync.lock = 0

        if err:
            self.resync = self.resync + 1

        if self.resync.lock:
            return False
        self.resync.lock = self.resync.lock + 1

        time.sleep(0.008)

        while self.serial.inWaiting():
            self.serial.read(self.serial.inWaiting())

        self.set_toggle_mode(1 - m)

        if ip != self.current_ip:
            if self.toggle_mode:
                cmd = bytearray()
                cmd.append(CMD_IDENTIFY)
                resp = self.raw(IPDONTCARE,cmd)
                if len(resp) == 0:
                    self.resync.lock = 0
                    return False
                if ord(resp[0]) & RESP_IDENTIFY_MASK != 1 - self.current_ip:
                    self.rtxerr = COMMS_NOT_INITIALISED
                    self.resync.lock = 0
                    return False
            else:
                self.current_ip = 1 - self.current_ip
                cmd = bytearray()
                cmd.append(CMD_TOGGLE_ONCE)
                resp = self.raw(IPDONTCARE, cmd)
                if len(resp) == 0:
                    self.resync.lock = 0
                    return False

        self.resync.lock = 0
        return True

    def init_comms(toggle_mode):
        ip = 0

        self.initialising = True
        self.initialised = False
        self.lasttime = current_millis()

        resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_OFF)))
        if len(resp) == 0:
            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_OFF)))
            if len(resp) == 0:
                return False

        if (len(resp) != 0 or
            (ord(resp[0]) != RESP_ACK and
             ord(resp[0]) != RESP_IP_RESTART and
             ord(resp[0]) != RESP_IP_RESTART1)):
            self.rtxerr = COMMS_FAULT
            return False

        resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
        if len(resp) == 0:
            return False

        if len(resp) != 1:
            self.rtxerr = RESPONSE_OVERRUN
            return False

        self.toggle_mode = 0

        if ord(resp[0]) == RESP_IP_RESTART or ord(resp[0]) == RESP_IP_RESTART1:
            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_GO)))
            if len(resp) == 0:
                return False
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                self.rtxerr = COMMS_FAULT
                return False
            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
            if len(resp) == 0:
                return False
            if (len(resp) != 1 or
                (ord(resp[0]) != RESP_IDENTIFY_0 and
                 ord(resp[0]) != RESP_IDENTIFY_1)):
                self.rtxerr = COMMS_FAULT
                return False
            ip = ord(resp[0]) & RESP_IDENTIFY_MASK
        elif ord(resp[0]) == RESP_IDENTIFY_0 or ord(resp[0]) == RESP_IDENTIFY_1:
            ip = ord(resp[0]) & RESP_IDENTIFY_MASK
        else:
            self.rtxerr = COMMS_FAULT
            return False

        resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ONCE)))
        if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
            self.rtxerr = COMMS_FAULT
            return False

        resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_OFF)))
        if len(resp) != 1 or (ord(resp[0]) != RESP_ACK and
                              ord(resp[0]) != RSP_IP_RESTART and
                              ord(resp[0]) != RSP_IP_RESTART1):
            self.rtxerr = COMMS_FAULT
            return False

        resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
        if len(resp) != 1:
            self.rtxerr = COMMS_FAULT
            return False

        if ord(resp[0]) == RESP_IP_RESTART or ord(resp[0]) == RESP_IP_RESTART1:
            resp = rtx_raw(IPDONTCARE, umi_cmd((CMD_GO)))
            if len(resp) == 0:
                return False
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                self.rtxerr = COMMS_FAULT
                return False
            resp2 = rtx_raw(IPDONTCARE, umi_cmd((CMD_IDENTIFY)))
            if len(resp) == 0:
                return False
            if (len(resp) != 1 or
                (ord(resp[0]) != RESP_IDENTIFY_0 and
                 ord(resp[0]) != RESP_IDENTIFY_1)):
                self.rtxerr = COMMS_FAULT
                return False
            self.current_ip = ord(resp[0]) & RESP_IDENTIFY_MASK
        elif ord(resp[0]) == RESP_IDENTIFY_0 or ord(resp[0]) == RESP_IDENTIFY_1:
            self.current_ip = ord(resp[0]) & RESP_IDENTIFY_MASK
        else:
            self.rtxerr = COMMS_FAULT
            return False

        if ip == self.current_ip:
            self.rtxerr = COMMS_FAULT
            return False

        if toggle_mode:
            resp = self.raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ON)))
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                self.rtxerr = COMMS_FAULT
                return False
            resp = self.raw(IPDONTCARE, umi_cmd((CMD_TOGGLE_ON)))
            if len(resp) != 1 or ord(resp[0]) != RESP_ACK:
                self.rtxerr = COMMS_FAULT
                return False

            self.toggle_mode = True

        self.initialising = False
        self.initialised = True

        return True


if __name__=='__main__':
    argv = sys.argv

    umi = None
    if len(argv) > 1:
        umi = UMI(argv[1])
    else:
        umi = UMI('/dev/ttyUSB0')

    umi.open()

    cmd = bytearray()
    cmd.append(CMD_IDENTIFY)

    # little hack
    resp = umi.comms_init(1)
    print 'got respons: %s' % (repr(resp))

