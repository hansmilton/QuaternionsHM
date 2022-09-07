A package that adds numeric and symbolic quaternion capabilities to Mathematica.<br />
Also some utilities, among them conversions to/from matrix and angle-axis representation.
## Scope
Primarily intended for practical and numerical work when rotating coordinate frames in 3D.<br />
But function arguments can also be symbolic. Or a mix of numeric and symbolic.<br />
An exception is the function quatToFromEulerZYX, which requires numeric arguments.

quat is the head of a quaternion expression.<br />
Syntax is quat[*q0, q1, q2, q3*].<br />
If a normalized quaternion is interpreted as a rotation then *q0* is cosine of half the rotation angle,<br />
while {*q1, q2, q3*} is a vector along the axis of rotation.

Functionality is added to 9 inbuilt functions:
- NonCommutativeMultiply, quat ** quat
- Power, quat<sup>scalar</sup>
- Conjugate
- Norm
- Normalize
- Exp
- Log
- Times, multiplication of quat with scalar
- Plus, addition of quat with scalar and quat with quat

In addition 6 new functions:
- quatToFromList
- quatToFromθV
- quatToFromMatrix
- quatRotateVector
- quatToFromEulerZYX
- quatFromAlignedMatrix
## Matrix representation
Any quaternion can be converted to a 3x3 matrix.<br />
There are two different conventions for representing rotations by matrices:
- Passive, or coordinate frame oriented
- Active, or vector oriented

The two conventions are transposes of each other.

This package uses the passive convention.<br />
The matrix rows are the base axes of a rotated frame, as seen from the reference frame.

In contrast, Mathematica's inbuilt function RotationMatrix uses the active convention.<br />
The matrix rows are the base axes of the reference frame, as seen from a rotated frame.

The passive convention matches the result of mapping the function quatRotateVector over<br />
the rows of an identity 3x3 matrix.
