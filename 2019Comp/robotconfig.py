{
  # Class names of motor controllers used.
  # Options:
  # 'Spark'
  # 'Victor'
  # 'VictorSP'
  # 'PWMTalonSRX'
  # 'PWMVictorSPX'
  # 'WPI_TalonSRX'
  # 'WPI_VictorSPX'
  'rightControllerTypes': ('WPI_TalonSRX', 'WPI_TalonSRX'),
  'leftControllerTypes': ('WPI_TalonSRX', 'WPI_TalonSRX'),

  # Ports for the left-side motors
  'leftMotorPorts': (2, 5),
  # Ports for the right-side motors
  'rightMotorPorts': (3, 6),

  # Inversions for the left-side motors
  'leftMotorsInverted': (False, False),
  # Inversions for the right side motors
  'rightMotorsInverted': (True, True),

  # Wheel diameter (in units of your choice - will dictate units of analysis)
  'wheelDiameter': .5,

  # If your robot has only one encoder, remove all of the right enacoder fields
  # Encoder pulses-per-revolution (*NOT* cycles per revolution!)
  # This value should be the pulses per revolution *of the wheels*, and so
  # should take into account gearing between the encoder and the wheels
  'encoderPPR': 120,
  # Ports for the left-side encoder
  'leftEncoderPorts': (0, 1),
  # Ports for the right-side encoder
  'rightEncoderPorts': (2, 3),
  # Whether the left encoder is inverted
  'leftEncoderInverted': False,
  # Whether the right encoder is inverted:
  'rightEncoderInverted': True,

  # Whether the robot should turn (for angular tests)
  'turn': False,
}

