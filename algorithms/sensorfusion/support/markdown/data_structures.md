Data Structures From a High Level         {#data_structures}
=================================
The V7.00 design kit never calls malloc().  All storage is allocated at compile time and defined statically in the calling program.  To make this easier, this version of the kit has in-lined structures which, in previous releases, were free-standing.
![DataStructures](DataStructures.png)

Notes
-----
* Bold black are in-line structures
* Bold red are function pointers
* Normal text = scalars and pointers
