# Car2 schematic review draft r01

This is a system wiring schematic, not a verified as-built drawing or PCB design.
The PDF contains three A3 landscape vector sheets. The matching SVG files are
editable. The CSV lists the known/intended nets and open questions.

Evidence labels: U = user report or live observation; R = repository/intended
wiring; ? = unknown, unverified or incomplete. Neither a repository statement nor
a firmware pin assignment proves the physical harness connection.

Caleb pin assignments are used throughout. The reference Paarseus repository was
used for comparison only: its steering STEP5/DIR4/limits20/21 and brake PWM12/11
are not substituted into this drawing. It also uses a different throttle DAC.

Known CAN cable order is acceleration -> master -> steering -> brake. Brake's
module remains on the bus when its Teensy is unplugged. Termination drawn is the
required target arrangement, not a claim that the resistors have been installed.
An earlier 30-ohm reading was reported; a corrected reading is not yet confirmed.

The replacement-master/steering CAN diagnostic round trip passed. Exact physical
damage to the earlier board has not been established. The four-node CAN test was
built and tested offline but has not been uploaded. Installed firmware/USB board
roles are changing during diagnostics, so the drawing identifies functional slots
rather than treating old COM assignments as permanent hardware identities.

The steering switch location/pin observations do not establish which travel
direction each contact protects. The reported failed steering encoder has not
been positively identified as the A0 position sensor versus the motor encoder.
These are drawn separately for review, rather than assuming they are one device.

Sources inspected:
- Local car2_ws/firmware/teensys/{master,throttle,steer,brake}.cpp and dbw_can.h
- Local car2_ws/README.md hardware notes (historical operating claims are not
  treated as current test evidence).
- User-provided physical cable order, TX22/RX23/3.3V/GND connections, module
  photograph, power description, steering movement and switch observations.
- https://github.com/calebhylkema/car2_ws
- https://github.com/Paarseus/av-control-system (comparison only)
- https://www.pjrc.com/store/teensy41.html#power
- https://www.pjrc.com/teensy/card11a_rev4_web.pdf
- https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf (termination and transceiver pins)

Review questions:
- Q01: CAN module chip/model; actual H/L screw-terminal labels and wire polarity.
- Q02: Which termination jumpers/resistors are fitted? Confirm 60 ohms powered off.
- Q03: Buck output(s), VIN wiring, ground bonding and each VIN-VUSB link state.
- Q04: Driver/controller model labels and all motor-supply voltages/fuse ratings.
- Q05: CL57T PUL+/PUL-, DIR+/DIR-, ENA connections and any interface circuit.
- Q06: Which pin17/pin18 switch is physically reached on left and right travel?
- Q07: IBT-2 PWM/enable/VCC terminals; actuator motor pair and feedback wiring.
- Q08: MCP4725 supply; Votol throttle ground/input and GPIO33/34/35 terminals.
- Q09: Physical E-stop/disconnect/contactor chain; underglow supply and switching.
- Q10: Which steering sensor failed? Motor encoder vs A0 feedback; encoder supplies.
