# CubeSat
Code for CubeSat motion control

Consists of the following classes.

* __class__ Satellite. A class for satellites in general.

    * __class__ CubeSat(Satellite). A subclass of Satellite for CubeSat.

        * __class__ ImpairedCubeSat(CubeSat). A subclass of CubeSat for a CubeSat that can't rotate and move directionally at the same time.

    * __class__ Target(Satellite).  A subclass of Satellite for the Target.

* __class__ Sim. The simulation infrastructure.

