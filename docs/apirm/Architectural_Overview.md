ISSDK Architectural Overview   {#arch}
=============================

<P>This chapter provides the architectural overview for the Iot sensing SDK 1.8. It
describes each layer within the architecture and its associated components.
<P><b>Overview</b>
<P>ISSDK is designed to provide separable layers of functionality that a customer can
choose to use or ignore based on their specific needs.  In addition, the ISSDK architecture
is portable across MCUXpresso SDK due to the use of open APIs (ARM Ltd.’s CMSIS Driver APIs).
ISSDK contains both Sensor Drivers to get at sensor data and algorithms to process the raw sensor data
(Sensor Fusion and Pedometer are supported). ISSDK is designed to allow users to start with
as small a production footprint (memory and CPU load) as is practical for their particular
application.  This is typically done by selecting the Bare Metal option,
however some applications may prefer using one of the RTOSs supplied with MCUXpresso SDK 2.x.
<P>Figure shows the high level “layer cake” architecture of the ISSDK 1.8 software.

@image html ISSDK_Architecture.jpg ISSDK 1.8 Architecture Block Diagram
@image latex ISSDK_Architecture.jpg ISSDK 1.8 Architecture Block Diagram
 