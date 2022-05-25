(* ::Package:: *)

(* ::Title:: *)
(*QuaternionsHM*)


(* ::Text:: *)
(*A package that adds numeric and symbolic quaternion capabilities to Mathematica. *)
(*Also some utilities, among them conversions to/from matrix and angle-axis representation.*)


(* ::Text:: *)
(*Author: Hans Milton*)


(* ::Section::Closed:: *)
(*Information*)


(* ::Subsection::Closed:: *)
(*Scope*)


(* ::Text:: *)
(*Primarily intended for practical and numerical work when rotating coordinate frames in 3D. *)
(*But function arguments can also be symbolic. Or a mix of  numeric and symbolic. *)
(*An exception is the function quatToFromEulerZYX, which requires numeric arguments.*)


(* ::Text:: *)
(*quat is the head of a quaternion expression.*)
(*Syntax is quat[q0, q1, q2, q3].*)
(*If a normalized quaternion is interpreted as a rotation then q0 is cosine*)
(*of half the rotation angle, while {q1, q2, q3} is a vector along the axis of rotation.*)


(* ::Text:: *)
(*Functionality is added to 9 inbuilt functions:*)
(*    NonCommutativeMultiply, quat \[Star]\[Star] quat*)
(*    Power[quat,scalar]*)
(*    Conjugate*)
(*    Norm*)
(*    Normalize*)
(*    Exp*)
(*    Log*)
(*    Times, multiplication of quat with scalar*)
(*    Plus, addition of quat with scalar and quat with quat*)


(* ::Text:: *)
(*In addition 6 new functions:*)
(*    quatToFromList*)
(*    quatToFrom\[Theta]V*)
(*    quatToFromMatrix*)
(*    quatRotateVector*)
(*    quatToFromEulerZYX*)
(*    quatFromAlignedMatrix*)


(* ::Subsection::Closed:: *)
(*Matrix representation*)


(* ::Text:: *)
(*Any quaternion can be converted to a 3x3 matrix.*)
(*There are two different conventions for representing rotations by matrices:*)
(*- Passive, or coordinate frame oriented*)
(*- Active, or vector oriented*)
(*The two conventions are transposes of each other.*)


(* ::Text:: *)
(*This package uses the passive convention. *)
(*The matrix rows are the base axes of a rotated frame, as seen from the reference frame.*)


(* ::Text:: *)
(*In contrast Mathematicas inbuilt function RotationMatrix use the active convention.*)
(*The matrix rows are the base axes of the reference frame, as seen from a rotated frame.*)


(* ::Subsection::Closed:: *)
(*History*)


(* ::Text:: *)
(*v1.0, April 2014. Mathematica 9.0.1.*)


(* ::Text:: *)
(*v2.0, November 2018. Mathematica 11.3.*)
(*Added quat functionality to Exp, Log, Power[quat,scalar] and Conjugate.*)
(*Added functions quatToFromEulerZYX and quatFromAlignedMatrix.*)


(* ::Text:: *)
(*v2.1, May 2021. Mathematica 12.2.*)
(*Added Times[scalar,quat].*)
(*Added Plus[scalar,quat] and Plus[quat,quat].*)
(*Updates to quatToFrom\[Theta]V with symbolic arguments.*)
(*Updates to quatToFromMatrix.*)


(* ::Text:: *)
(*v2.2, May 2022. Mathematica 12.2.*)
(*General overhaul.*)


(* ::Section::Closed:: *)
(*Begin Package*)


BeginPackage["QuaternionsHM`"]


(* ::Section::Closed:: *)
(*Export of public functions by usage texts*)


quat::usage=
"quat is the head of a quaternion expression. 
Syntax is quat[\!\(\*
StyleBox[\"q0\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"q1\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"q2\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"q3\",\nFontSlant->\"Italic\"]\)].
If a normalized quaternion is interpreted as a rotation then \!\(\*
StyleBox[\"q0\",\nFontSlant->\"Italic\"]\) is cosine
of half the rotation angle, while {\!\(\*
StyleBox[\"q1\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"q2\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"q3\",\nFontSlant->\"Italic\"]\)} is a vector along the axis of rotation.
Functionality for quat is added to 9 inbuilt functions:
-  Non commutative multiplication, quat ** quat.
-  Power, \!\(\*SuperscriptBox[\(quat\), \(p\)]\). The exponent p has to be a scalar.
-  Conjugate
-  Norm
-  Normalize
-  Exp
-  Log
-  Times, multiplication of quat with scalar
-  Plus, addition of quat with scalar and quat with quat";


quatToFromList::usage=
"Converts a quaternion to list, or vice verse.

quatToFromList[\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)]. Returns list.
quatToFromList[\!\(\*
StyleBox[\"list\",\nFontSlant->\"Italic\"]\)]. Returns quaternion.";


quatToFrom\[Theta]V::usage=
"Conversion of the representation of a rotation. Between quaternion and angle / axis of rotation.
The axis of rotation is a 3D vector. As input it can have any length. As output it has unit length.
Symbolic input of quaternion or axis vector is assumed to be normalized.

quatToFrom\[Theta]V[\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)]. Returns {angle,vector}.
quatToFrom\[Theta]V[\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"vector\",\nFontSlant->\"Italic\"]\)]. Returns quaternion.
quatToFrom\[Theta]V[{\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"vector\",\nFontSlant->\"Italic\"]\)}]. Returns quaternion.";


quatToFromMatrix::usage=
"Converts a quaternion to matrix, or vice verse.
An input matrix where all elements are numeric has to be righthanded.
An input matrix where all elements are explicit numbers does not have to be normalized.

quatToFromMatrix[\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)]. Returns matrix.
quatToFromMatrix[\!\(\*
StyleBox[\"matrix\",\nFontSlant->\"Italic\"]\)]. Returns quaternion";


quatRotateVector::usage=
"Rotation of a 3D vector, with rotation expressed as quaternion.

quatRotateVector[\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"vector\",\nFontSlant->\"Italic\"]\)]. Returns rotated vector.";


quatToFromEulerZYX::usage=
"Converts a quaternion to Euler ZYX angles, or vice verse.
Numeric input only. Input and output angles in decimal degrees.

quatToFromEulerZYX[\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)]. Returns a table with the possible angle sequences.
quatToFromEulerZYX[\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"z\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"y\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"x\",\nFontSlant->\"Italic\"]\)]. Returns quaternion.
quatToFromEulerZYX[{\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"z\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"y\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\",\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"angle\",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\" \",\nFontSlant->\"Italic\"]\)\!\(\*
StyleBox[\"x\",\nFontSlant->\"Italic\"]\)}]. Returns quaternion.";


quatFromAlignedMatrix::usage=
"A matrix is selected by interactive dialogs and converted to quaternion.
Matrices that can be selected represent a rotated coordinate frame where all axes
are colinear with axes of the reference frame. Matrix elements from the set {-1,0,1}.

quatFromAlignedMatrix[ ]. No arguments. Returns quaternion.";


(* ::Section::Closed:: *)
(*Begin Private*)


Begin["`Private`"]


(* ::Subsection::Closed:: *)
(*Upvalues for quat*)


quat/:NonCommutativeMultiply[p:quat[p0_,p1_,p2_,p3_],q:quat[q0_,q1_,q2_,q3_]]/;quatQ[p]&&quatQ[q]:=
Module[
  {pV,qV,r0,rV},
  pV={p1,p2,p3}; qV={q1,q2,q3};
  r0=p0 q0-Dot[pV,qV];
  rV=p0 qV+q0 pV+Cross[pV,qV];
  Prepend[rV,r0]//qOut
]


quat/:Power[base_quat,exponent_]/;quatQ[base]&&scalarQ[exponent]:=Exp[exponent*Log@base]


quat/:Conjugate[q:quat[q0_,q1_,q2_,q3_]]/;quatQ[q]:=quat[q0,-q1,-q2,-q3]


quat/:Norm[q_quat]/;quatQ[q]:=Norm[List@@q]


quat/:Normalize[q_quat]/;quatQ[q]:=Normalize[List@@q]//qOut


quat/:Exp[q_quat]/;quatQ[q]:=With[
  {q0=First@q,qV=Rest[List@@q]},
  E^q0 {Cos[Norm@qV],Normalize[qV] Sin[Norm@qV]}//Flatten//qOut
]


quat/:Log[q_quat]/;quatQ[q]:=With[
  {q0=First@q,qV=Rest[List@@q]},
  {Log@Simplify@Norm[q], Normalize[qV] ArcCos[q0/Norm[q]]}//Flatten//quat@@#&//Chop
]


quat/:Times[s_,q_quat]/;scalarQ[s]&&quatQ[q]:=s*#&/@q


quat/:Plus[s_,q_quat]/;scalarQ[s]&&quatQ[q]:=s+#&/@q
quat/:Plus[p_quat,q_quat]/;quatQ[p]&&quatQ[q]:=Apply[quat,List@@p+List@@q]


quat/:Times[NonCommutativeMultiply[p_quat,1],q_quat]:=p**q


(* ::Subsection::Closed:: *)
(*Public functions*)


quatToFromList[q_?quatQ]:=List@@q
quatToFromList[l_?listQ]:=l//qOut


quatToFrom\[Theta]V[q_?quatQ]/;Norm@Chop@Take[List@@q,-3]===0:={0,{0,0,0}}
quatToFrom\[Theta]V[q:quat[q0_,__]?quatQ]/;And[NumericQ@q0,symvectQ[Take[q,-3]],Abs[q0]>=1]:=
  {0,{0,0,0}}
quatToFrom\[Theta]V[q_?quatQ]/;symvectQ[Take[q,-3]]:=
  {2 ArcCos[First@q],Take[List@@q,-3]/Sqrt[1-First[q]^2]}//\[Theta]VOut
quatToFrom\[Theta]V[q_?quatQ]:=
  {2 ArcCos[First@q/Norm@q],Normalize@Take[List@@q,-3]}//\[Theta]VOut

quatToFrom\[Theta]V[PatternSequence[\[Theta]_?scalarQ,v_?vectQ]|{\[Theta]_?scalarQ,v_?vectQ}]/;Norm@Chop@v===0:=
  quat[1,0,0,0]
quatToFrom\[Theta]V[PatternSequence[\[Theta]_?scalarQ,v_?vectQ]|{\[Theta]_?scalarQ,v_?vectQ}]/;symvectQ[v]:=
  {Cos[\[Theta]/2],Sin[\[Theta]/2] v}//Flatten//qOut
quatToFrom\[Theta]V[PatternSequence[\[Theta]_?scalarQ,v_?vectQ]|{\[Theta]_?scalarQ,v_?vectQ}]:=
  {Cos[\[Theta]/2],Sin[\[Theta]/2] Normalize@v}//Flatten//qOut


quatToFromMatrix[q_?quatQ/;qType@q==40]:=mBasicFromQ[q]
quatToFromMatrix[quat[q0_,q1_,q2_,q3_]?quatQ]:=
  {
    {q0^2+q1^2-q2^2-q3^2,2 (q0 q3+q1 q2),2 (q1 q3-q0 q2)},
    {2 (q1 q2-q0 q3),q0^2-q1^2+q2^2-q3^2,2 (q0 q1+q2 q3)},
    {2 (q0 q2+q1 q3),2 (q2 q3-q0 q1),q0^2-q1^2-q2^2+q3^2}
  }//mOut

quatToFromMatrix[m_?matQ]:=Switch[mType@m,
  1,qFromNumberM[m],
  2,qFromNumericM[m],
  3,qFromSymbolicM[m],
  4,qFromSymbolicM[m],
  21,qFromNumeric180\[Degree]M[m],
  33,qFromSymbolicIdentityM[m],
  34,qFrom90\[Degree]AroundBaseAxisM[m],
  35,qFrom120\[Degree]AroundOctantDiagonalM[m],
  36,qFrom180\[Degree]AroundBaseAxisM[m],
  37,qFrom180\[Degree]AroundPlaneDiagonalM[m],
  38,qFrom180\[Degree]AroundAxisInBasePlaneM[m],
  40,qBasicFromM[m],
  48,qFrom180\[Degree]AroundAxisOutOfBasePlaneM[m],
  _,m
]


quatRotateVector[q_?quatQ,v_?vectQ]:=Module[
  {q0,qV,r0,rV},
  q0=First@q;
  qV=Take[List@@q,-3];
  r0=Dot[qV,v];
  rV=q0*v+Cross[qV,v];
  q0*rV+r0*qV+Cross[rV,-qV]//vOut
]


quatToFromEulerZYX[q_?quatQ/;AllTrue[q,NumericQ]]:=Module[
  {m,singularity},
  m=q//N//Normalize//quatToFromMatrix;
  singularity=If[Abs@m[[1,3]]==1,True,False];
  TableForm[
    eulerZYXangles[m,singularity],
    TableAlignments->Right,TableHeadings->{None,{"Z","Y","X"}}
  ]
]

quatToFromEulerZYX[PatternSequence[\[Alpha]_?NumericQ,\[Beta]_?NumericQ,\[Gamma]_?NumericQ]|{\[Alpha]_?NumericQ,\[Beta]_?NumericQ,\[Gamma]_?NumericQ}]:=
  quatToFrom\[Theta]V[N@\[Alpha] \[Degree],{0,0,1}]**quatToFrom\[Theta]V[N@\[Beta] \[Degree],{0,1,0}]**quatToFrom\[Theta]V[N@\[Gamma] \[Degree],{1,0,0}]


quatFromAlignedMatrix[]:=N@quatToFromMatrix[alignedMatrixDialogs[]]/.{0.->0,1.->1}


(* ::Subsection::Closed:: *)
(*Lower level functions*)


(* ::Subsubsection::Closed:: *)
(*Checking arguments to public functions*)


scalarQ[e_]:=ReplaceAll[e,s_Symbol/;!Context@s==="System`":>RandomReal[]]//NumericQ


listQ[l_]:=And[Head@l===List,scalarQ[#]&/@l==={True,True,True,True}]


vectQ[v_]:=And[Head@v===List,scalarQ[#]&/@v==={True,True,True}]


quatQ[q_]:=And[Head@q===quat,scalarQ[#]&/@q===quat[True,True,True,True]]


matQ[m_]:=And[Head@m===List,Dimensions@m==={3,3},Map[scalarQ,m,{2}]===ConstantArray[True,{3,3}]]


symvectQ[v_]:=And[Count[Chop@v,0|(e_/;!NumericQ[e])]==3,Count[Chop@v,0]<3]


(* ::Subsubsection::Closed:: *)
(*Type codes*)


(* ::Text:: *)
(*    0:  Nonvalid*)
(*    1:  All explicit numbers*)
(*    2:  All numeric*)
(*    21: Numeric 180\[Degree] rotation matrix*)
(*    3:  Mixed numeric and symbolic*)
(*    33: Symbolic identity matrix*)
(*    34: Symbolic matrix representing 90\[Degree] rotation around a base axis*)
(*    35: Symbolic matrix representing 120\[Degree] rotation around an octant diagonal*)
(*    36: Symbolic matrix representing 180\[Degree] rotation around a base axis*)
(*    37: Symbolic matrix representing 180\[Degree] rotation around a base plane diagonal*)
(*    38: Symbolic matrix representing 180\[Degree] rotation around axis in a base plane*)
(*    39: Quat from symbolic single angle matrix, rotation axis in a base plane*)
(*    4:  All symbolic*)
(*    40: Quat or matrix from basic symbolic function-free matrix or quat*)
(*    41: Quat from Euler 3 symbolic angles matrix*)
(*    42: Quat from Euler 2 symbolic angles matrix*)
(*    48: Symbolic matrix representing 180\[Degree] rotation around axis out of base planes*)
(*    49: Quat from symbolic single angle matrix, rotation axis out of base planes*)


(* ::Subsubsection::Closed:: *)
(*Type of quat*)


qType[q_quat]:=Module[
  {type},
  type=Which[
    AllTrue[q,NumberQ],1,
    AllTrue[q,NumericQ],2,
    AnyTrue[q,NumericQ],3,
    True,4
  ];
  type=Switch[type,
    1|2,type,
    3,Which[
        isQuatType39[q],39,
        True,3
      ],
    4,Which[
        isQuatType40[q],40,
        isQuatType41[q],41,
        isQuatType42[q],42,
        isQuatType49[q],49,
        True,4
      ]
  ];
  type
]


isQuatType39[q_]:=Or[isQuatType39HA@q,isQuatType39FA@q]


isQuatType39HA[q_]:=With[
  {
  cmnA=Sqrt[2-a_Symbol^2+Cos[\[Theta]_Symbol]+a_Symbol^2*Cos[\[Theta]_Symbol]+1/2*(1+a_Symbol^2+Cos[\[Theta]_Symbol]-a_Symbol^2*Cos[\[Theta]_Symbol])],
  cmnB=Sqrt[5-x_Symbol^2-y_Symbol^2+(3+x_Symbol^2+y_Symbol^2)*Cos[\[Theta]_Symbol]],
  cmnC=Sqrt[1+2*Cos[\[Theta]_Symbol/2]^2+1/2*(1-2*a_Symbol^2+Cos[\[Theta]_Symbol]+2*a_Symbol^2*Cos[\[Theta]_Symbol])]
  },
  And[
    MemberQ[{1,2},Count[q,0]],
    Or[
      MatchQ[q,quat[1/2*cmnA,Repeated[0|a_Symbol*Sin[\[Theta]_Symbol]/cmnA|-a_Symbol*Sin[\[Theta]_Symbol]/cmnA]]],
      MatchQ[q,quat[cmnB/(2*Sqrt[2]),Repeated[0|Sqrt[2]*_Symbol*Sin[\[Theta]_Symbol]/cmnB|-Sqrt[2]*_Symbol*Sin[\[Theta]_Symbol]/cmnB]]],
      MatchQ[q,quat[1/2*cmnC,Repeated[0|a_Symbol*Sin[\[Theta]_Symbol]/cmnC|-a_Symbol*Sin[\[Theta]_Symbol]/cmnC]]]
    ]
  ]
]


isQuatType39FA[q_]:=With[
  {
  cmnA=Sqrt[1+3*Cos[\[Theta]_Symbol]^2-a_Symbol^2*Sin[\[Theta]_Symbol]^2],
  cmnB=Sqrt[1+3*Cos[\[Theta]_Symbol]^2+(x_Symbol^2-y_Symbol^2)*Sin[\[Theta]_Symbol]^2+(-x_Symbol^2+y_Symbol^2)*Sin[\[Theta]_Symbol]^2-(x_Symbol^2+y_Symbol^2)*Sin[\[Theta]_Symbol]^2],
  cmnC=Sqrt[1+3*Cos[\[Theta]_Symbol]^2-2*a_Symbol^2*Sin[\[Theta]_Symbol]^2],
  qVnumAC=2*a_Symbol*Cos[\[Theta]_Symbol]*Sin[\[Theta]_Symbol]+a_Symbol*Sin[2*\[Theta]_Symbol],
  qVnumB=2*_Symbol*Cos[\[Theta]_Symbol]*Sin[\[Theta]_Symbol]+_Symbol*Sin[2*\[Theta]_Symbol]
  },
  And[
    MemberQ[{1,2},Count[q,0]],
    Or[
      MatchQ[q,quat[1/2*cmnA,Repeated[0|qVnumAC/(2*cmnA)|-qVnumAC/(2*cmnA)]]],
      MatchQ[q,quat[1/2*cmnB,Repeated[0|qVnumB/(2*cmnB)|-qVnumB/(2*cmnB)]]],
      MatchQ[q,quat[1/2*cmnC,Repeated[0|qVnumAC/(2*cmnC)|-qVnumAC/(2*cmnC)]]]
    ]
  ]
]


isQuatType40[q_]:=Module[
  {ptrn,theform},
  ptrn[e_]:=MatchQ[e,_Symbol|Subscript[_Symbol,_]];
  theform=quat[
    1/2 Sqrt[1 + e11_?ptrn + e22_?ptrn+ e33_?ptrn], 
    (e23_?ptrn- e32_?ptrn)/(2 Sqrt[1 + e11_?ptrn + e22_?ptrn+ e33_?ptrn]), 
    (-e13_?ptrn + e31_?ptrn)/(2 Sqrt[1 + e11_?ptrn + e22_?ptrn+ e33_?ptrn]),
    (e12_?ptrn- e21_?ptrn)/(2 Sqrt[1 + e11_?ptrn + e22_?ptrn + e33_?ptrn])
  ];
  MatchQ[q,theform]
]


isQuatType41[q_]:=And[
  Length[Cases[q,_Symbol,-1]//DeleteDuplicates]==3,
  Or[isQuatType41HA@q,isQuatType41FA@q]
]


isQuatType41HA[q_]:=With[
  {
  cmnA=Sqrt[1+Cos[\[Alpha]_Symbol] Cos[\[Beta]_Symbol]+Cos[\[Alpha]_Symbol] Cos[\[Gamma]_Symbol]+Cos[\[Beta]_Symbol] Cos[\[Gamma]_Symbol]-Sin[\[Alpha]_Symbol] Sin[\[Beta]_Symbol] Sin[\[Gamma]_Symbol]],
  cmnB=Sqrt[1+Cos[\[Alpha]_Symbol] Cos[\[Beta]_Symbol]+Cos[\[Alpha]_Symbol] Cos[\[Gamma]_Symbol]+Cos[\[Beta]_Symbol] Cos[\[Gamma]_Symbol]+Sin[\[Alpha]_Symbol] Sin[\[Beta]_Symbol] Sin[\[Gamma]_Symbol]],
  cmnC=Sqrt[1+Cos[\[Beta]_Symbol]+Cos[\[Alpha]_Symbol] Cos[\[Gamma]_Symbol]+Cos[\[Alpha]_Symbol] Cos[\[Beta]_Symbol] Cos[\[Gamma]_Symbol]-Sin[\[Alpha]_Symbol] Sin[\[Gamma]_Symbol]-Cos[\[Beta]_Symbol] Sin[\[Alpha]_Symbol] Sin[\[Gamma]_Symbol]],
  cmnD=Sqrt[1+Cos[\[Beta]_Symbol]+Cos[\[Alpha]_Symbol] Cos[\[Gamma]_Symbol]+Cos[\[Alpha]_Symbol] Cos[\[Beta]_Symbol] Cos[\[Gamma]_Symbol]+Sin[\[Alpha]_Symbol] Sin[\[Gamma]_Symbol]+Cos[\[Beta]_Symbol] Sin[\[Alpha]_Symbol] Sin[\[Gamma]_Symbol]]
  },
  And[
    Length[Cases[q,_Symbol,-1]//DeleteDuplicates]==3,
    Or[
      {Count[q,Cos[_Symbol],-1],Count[q,Sin[_Symbol],-1]}=={32,24},
      {Count[q,Cos[_Symbol],-1],Count[q,Sin[_Symbol],-1]}=={36,26}
    ],
    Or[
      MatchQ[q,quat[1/2*cmnA,_/(2*cmnA),_/(2*cmnA),_/(2*cmnA)]],
      MatchQ[q,quat[1/2*cmnB,_/(2*cmnB),_/(2*cmnB),_/(2*cmnB)]],
      MatchQ[q,quat[1/2*cmnC,_/(2*cmnC),_/(2*cmnC),_/(2*cmnC)]],
      MatchQ[q,quat[1/2*cmnD,_/(2*cmnD),_/(2*cmnD),_/(2*cmnD)]]
    ]
  ]
]


isQuatType41FA[q_]:=Module[
  {\[Alpha]1=\[Alpha]_Symbol,\[Beta]1=\[Beta]_Symbol,\[Gamma]1=\[Gamma]_Symbol,\[Alpha]2=\[Alpha]_Symbol*2,\[Beta]2=\[Beta]_Symbol*2,\[Gamma]2=\[Gamma]_Symbol*2,cmnA,cmnB,cmnC,cmnD,cmnE,cmnF},
  cmnA=Sqrt[1+Cos[\[Alpha]2] Cos[\[Beta]2]+Cos[\[Alpha]1]^2 Cos[\[Gamma]2]+Cos[\[Beta]2] Cos[\[Gamma]2]-Cos[\[Gamma]2] Sin[\[Alpha]1]^2-8 Cos[\[Alpha]1] Cos[\[Beta]1] Cos[\[Gamma]1] Sin[\[Alpha]1] Sin[\[Beta]1] Sin[\[Gamma]1]];
  cmnB=Sqrt[1+Cos[\[Alpha]2] Cos[\[Beta]2]+Cos[\[Alpha]1]^2 Cos[\[Gamma]2]+Cos[\[Beta]2] Cos[\[Gamma]2]-Cos[\[Gamma]1]^2 Sin[\[Alpha]1]^2+Sin[\[Alpha]1]^2 Sin[\[Gamma]1]^2+Sin[\[Alpha]2] Sin[\[Beta]2] Sin[\[Gamma]2]];
  cmnC=Sqrt[1+Cos[\[Beta]1]^2 Cos[\[Gamma]2]+Cos[\[Alpha]2] (Cos[\[Beta]2]+Cos[\[Gamma]2])-Cos[\[Gamma]2] Sin[\[Beta]1]^2-8 Cos[\[Alpha]1] Cos[\[Beta]1] Cos[\[Gamma]1] Sin[\[Alpha]1] Sin[\[Beta]1] Sin[\[Gamma]1]];
  cmnD=Sqrt[1+Cos[\[Beta]1]^2 Cos[\[Gamma]2]+Cos[\[Alpha]2] (Cos[\[Beta]2]+Cos[\[Gamma]2])-Cos[\[Gamma]1]^2 Sin[\[Beta]1]^2+Sin[\[Beta]1]^2 Sin[\[Gamma]1]^2+Sin[\[Alpha]2] Sin[\[Beta]2] Sin[\[Gamma]2]];
  cmnE=Sqrt[Cos[\[Beta]1]^2 Cos[\[Alpha]1+\[Gamma]1]^2];
  cmnF=Sqrt[Cos[\[Beta]1]^2 Cos[\[Alpha]1-\[Gamma]1]^2];
  Or[
    MatchQ[q,quat[1/2*cmnA,Repeated[_/(2*cmnA)|_/(4*cmnA)]]],
    MatchQ[q,quat[1/2*cmnB,Repeated[_/(2*cmnB)|_/(4*cmnB)]]],
    MatchQ[q,quat[1/2*cmnC,Repeated[_/(2*cmnC)]]],
    MatchQ[q,quat[1/2*cmnD,Repeated[_/(2*cmnD)]]],
    MatchQ[q,quat[cmnE,Repeated[_/(-4*cmnE)|_/(-2*cmnE)|_/(2*cmnE)|_/(4*cmnE)]]],
    MatchQ[q,quat[cmnF,Repeated[_/(-4*cmnF)|_/(-2*cmnF)|_/(2*cmnF)|_/(4*cmnF)]]]
  ]
]


isQuatType42[q_]:=Module[
  {\[Alpha]=\[Alpha]_Symbol|\[Alpha]_Symbol*2,\[Beta]=\[Beta]_Symbol|\[Beta]_Symbol*2,cmn,qVa,qVb,qVc,qVd},
  cmn=Sqrt[1+Cos[\[Alpha]]+Cos[\[Beta]]+Cos[\[Alpha]]*Cos[\[Beta]]];
  qVa=(Sin[\[Alpha]]+Cos[\[Beta]]*Sin[\[Alpha]])/(2*cmn);
  qVb=(-Sin[\[Alpha]]-Cos[\[Beta]]*Sin[\[Alpha]])/(2*cmn);
  qVc=(Sin[\[Alpha]]*Sin[\[Beta]])/(2*cmn);
  qVd=2*Cos[\[Alpha]_Symbol]*Cos[\[Beta]_Symbol]*Sin[\[Alpha]_Symbol]*Sin[\[Beta]_Symbol]/cmn;
  And[
    Length[Cases[q,_Symbol,-1]//DeleteDuplicates]==2,
    Count[q,Sin[\[Alpha]],-1]==6,
    MemberQ[{18,20},Count[q,Cos[\[Alpha]],-1]],
    MatchQ[q[[1]],1/2*cmn],
    And@@Map[MatchQ[#,qVa|qVb|qVc|-qVc|qVd|-qVd]&,Take[q,-3]]
  ]
]


isQuatType49[q_]:=Or[isQuatType49HA@q,isQuatType49FA@q]


isQuatType49HA[q_]:=With[
  {
  cmnA=Sqrt[5-x_Symbol^2-y_Symbol^2-z_Symbol^2+(3+x_Symbol^2+y_Symbol^2+z_Symbol^2)*Cos[\[Theta]_Symbol]],
  cmnB=Sqrt[5-2*a_Symbol^2-z_Symbol^2+(3+2*a_Symbol^2+z_Symbol^2)*Cos[\[Theta]_Symbol]],
  q0C=1/2*Sqrt[1+3*(1-a_Symbol^2+(1+a_Symbol^2)*Cos[\[Theta]_Symbol])/2],
  qVC=2*a_Symbol*Sin[\[Theta]_Symbol]/Sqrt[10-6*a_Symbol^2+6*(1+a_Symbol^2)*Cos[\[Theta]_Symbol]]
  },
  Or[
    MatchQ[q,quat[cmnA/(2*Sqrt[2]),Repeated[Sqrt[2]*_Symbol*Sin[\[Theta]_Symbol]/cmnA|-Sqrt[2]*_Symbol*Sin[\[Theta]_Symbol]/cmnA]]],
    MatchQ[q,quat[cmnB/(2*Sqrt[2]),Repeated[Sqrt[2]*_Symbol*Sin[\[Theta]_Symbol]/cmnB|-Sqrt[2]*_Symbol*Sin[\[Theta]_Symbol]/cmnB]]],
    MatchQ[q,quat[q0C,Repeated[qVC|-qVC]]]
  ]
]


isQuatType49FA[q_]:=With[
  {
  cmnA=Sqrt[1+3*Cos[\[Theta]_Symbol]^2-(x_Symbol^2+y_Symbol^2+z_Symbol^2)*Sin[\[Theta]_Symbol]^2],
  cmnB=Sqrt[1+3*Cos[\[Theta]_Symbol]^2-(2*a_Symbol^2+z_Symbol^2)*Sin[\[Theta]_Symbol]^2],
  cmnC=Sqrt[1+3*Cos[\[Theta]_Symbol]^2-3*a_Symbol^2*Sin[\[Theta]_Symbol]^2],
  qVnumAB1=_Symbol*Sin[2*\[Theta]_Symbol],
  qVnumAB2=2*_Symbol*Cos[\[Theta]_Symbol]*Sin[\[Theta]_Symbol],
  qVnumC1=2*a_Symbol*Sin[\[Theta]_Symbol]*(Cos[\[Theta]_Symbol]-a_Symbol*Sin[\[Theta]_Symbol])+2*a_Symbol*Sin[\[Theta]_Symbol]*(Cos[\[Theta]_Symbol]+a_Symbol*Sin[\[Theta]_Symbol]),
  qVnumC2=2*a_Symbol*Sin[\[Theta]_Symbol]*(-Cos[\[Theta]_Symbol]+a_Symbol*Sin[\[Theta]_Symbol])-2*a_Symbol*Sin[\[Theta]_Symbol]*(Cos[\[Theta]_Symbol]+a_Symbol*Sin[\[Theta]_Symbol])
  },
  Or[
    MatchQ[q,quat[1/2*cmnA,Repeated[qVnumAB1/cmnA|-qVnumAB2/cmnA]]],
    MatchQ[q,quat[1/2*cmnB,Repeated[qVnumAB1/cmnB|-qVnumAB2/cmnB]]],
    MatchQ[q,quat[1/2*cmnC,Repeated[qVnumC1/(2*cmnC)|-qVnumC1/(2*cmnC)|qVnumC2/(2*cmnC)|-qVnumC2/(2*cmnC)]]]
  ]
]


(* ::Subsubsection::Closed:: *)
(*Type of matrix*)


mType[m_]:=Module[
  {type},
  type=Which[
    MatrixQ[m,NumberQ],1,
    MatrixQ[m,NumericQ],2,
    AnyTrue[m,NumericQ,2],3,
    True,4
  ];
  type=Switch[type,
    1,Which[
        isNotRHMatrix[m],0,
        True,1
      ],
    2,Which[
        isNotRHMatrix[m],0,
        is180\[Degree]Matrix[m],21,
        True,2
      ],
    3,Which[
        isMatrixWithEmptyAxes[m],0,
        isLHAlignedMatrix[m],0,
        isMatrixType33[m],33,
        isMatrixType34[m],34,
        isMatrixType35[m],35,
        isMatrixType36[m],36,
        isMatrixType37[m],37,
        isMatrixType38[m],38,
        True,3
      ],
    4,Which[
        isInvalidSymbolicMatrix[m],0,
        isMatrixType40[m],40,
        isMatrixType48[m],48,
        True,4
      ]
  ];
  type
]


isNotRHMatrix[m_]:=Module[
  {nrh},
  nrh=Chop@Dot[Cross[m[[1]],m[[2]]],m[[3]]]<=0;
  If[nrh,Print@Framed["Not a righthanded matrix",FrameStyle->Red]];
  nrh
]


is180\[Degree]Matrix[mIn_]:=With[
  {m=mIn/.{0.->0,1.->1}},
  And[
    m[[2,3]]===m[[3,2]],
    m[[3,1]]===m[[1,3]],
    m[[1,2]]===m[[2,1]]
  ]
]


isMatrixWithEmptyAxes[mIn_]:=Module[
  {m=Chop[mIn],nullaxis={0,0,0},ea},
  ea=Or@@Join[MatchQ[#,nullaxis]&/@m,MatchQ[#,nullaxis]&/@Transpose[m]];
  If[ea,Print@Framed["Matrix with empty row or column",FrameStyle->Red]];
  ea
]


isAlignedMatrix[m_]:=And[
  Count[Flatten@m,Except[0|0.]]==3,
  SameQ@@Abs@Cases[Flatten@m,Except[0|0.]],
  Count[Flatten@m,e_/;!NumericQ[e]]==3
]


isLHAlignedMatrix[m_]:=Module[
  {am,lh=False},
  am=isAlignedMatrix[m];
  If[am,
    lh=Switch[Count[Diagonal@m,Except[0|0.]],
      0|3,lh=Replace[Times@@Cases[Flatten@m,Except[0|0.]]//First//Negative,Negative[_]->False],
      1,lh=Replace[Times@@Cases[Flatten@m,Except[0|0.]]//First//Positive,Positive[_]->True],
      _,True
    ];
    If[lh,Print@Framed["Lefthanded aligned matrix",FrameStyle->Red]]
  ];
  am&&lh
]


isMatrixType33[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,Except[0|0.]]==3,
  Replace[Times@@Diagonal[m]//First//Positive,Positive[_]->True],
  SameQ@@Diagonal[m]
]


isMatrixType34[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,Except[0|0.]]==1,
  UnsameQ@@Cases[Flatten@ReplacePart[m,{i_,i_}->0],Except[0|0.]],
  Replace[Times@@Cases[Flatten@m,Except[0|0.]]//First//Negative,Negative[_]->False]
]


isMatrixType35[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,0|0.]==3,
  Replace[Times@@Cases[Flatten@m,Except[0|0.]]//First//Positive,Positive[_]->True]
]


isMatrixType36[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,Except[0|0.]]==3,
  Replace[Times@@Diagonal[m]//First//Positive,Positive[_]->True],
  !SameQ@@Diagonal[m]
]


isMatrixType37[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,Except[0|0.]]==1,
  SameQ@@Cases[Flatten@ReplacePart[m,{i_,i_}->0],Except[0|0.]],
  Replace[Times@@Cases[Flatten@m,Except[0|0.]]//First//Negative,Negative[_]->False]
]


isMatrixType38[m_]:=And[
  is180\[Degree]Matrix[m],
  MatchQ[Diagonal@m,{Repeated[x_^2-y_^2|-x_^2+y_^2|-x_^2-y_^2]}],
  UnsameQ@@Diagonal[m],
  MatchQ[Cases[ReplacePart[m,{i_,i_}->Nothing],Except[0|0.],{2}],{2*x_*y_,2*x_*y_}|{-2*x_*y_,-2*x_*y_}]
]


isInvalidSymbolicMatrix[m_]:=Module[
  {chkA,chkB=False},
  chkA=Nand[DuplicateFreeQ@m,DuplicateFreeQ@Transpose@m];
  If[!chkA&&MatchQ[m,ConstantArray[_Symbol,{3,3}]],
    chkB=!OrderedQ[Diagonal@m,OrderedQ[{#1,#2}]&&UnsameQ[#1,#2]&]
  ];
  If[chkA,Print@Framed["Matrix with duplicated rows or columns",FrameStyle->Red]];
  If[chkB,Print@Framed["Symbols in matrix diagonal are not in order",FrameStyle->Red]];
  Or[chkA,chkB]
]


isMatrixType40[m_]:=With[
  {
  diaA=q0_Symbol^2+q1_Symbol^2-q2_Symbol^2-q3_Symbol^2,
  diaB=q0_Symbol^2-q1_Symbol^2+q2_Symbol^2-q3_Symbol^2,
  diaC=q0_Symbol^2-q1_Symbol^2-q2_Symbol^2+q3_Symbol^2
  },
  MatchQ[Diagonal@m,{diaA,diaB,diaC}]
]


isMatrixType48[m_]:=With[
  {eA=(-2|2)*x_*y_,eB=eA=(-2|2)*x_*z_,eC=eA=(-2|2)*y_*z_},
  And[
    is180\[Degree]Matrix[m],
    MatchQ[Diagonal@m,{Repeated[x_^2-y_^2-z_^2|-x_^2+y_^2-z_^2|-x_^2-y_^2+z_^2]}],
    UnsameQ@@Diagonal[m],
    MatchQ[ReplacePart[m,{i_,i_}->Nothing],{Repeated[{eA|eB|eC,eA|eB|eC}]}],
    UnsameQ@@ReplacePart[m,{i_,i_}->Nothing],
    And@@UnsameQ@@@ReplacePart[m,{i_,i_}->Nothing]
  ]
]


(* ::Subsubsection::Closed:: *)
(*quatToFromMatrix, quat input*)


mBasicFromQ[q:quat[q0_,q1_,q2_,q3_]]:=Module[
  {m=IdentityMatrix@3,qV,trace},
  qV=Numerator/@Take[q,-3];
  m[[2,3]]=First@Cases[qV[[1]],Except[-_]];
  m[[3,2]]=-First@Cases[qV[[1]],-_];
  m[[3,1]]=First@Cases[qV[[2]],Except[-_]];
  m[[1,3]]=-First@Cases[qV[[2]],-_];
  m[[1,2]]=First@Cases[qV[[3]],Except[-_]];
  m[[2,1]]=-First@Cases[qV[[3]],-_];
  trace=Part[(Last@First@q)^2,2;;4];
  m[[1,1]]=trace[[1]];
  m[[2,2]]=trace[[2]];
  m[[3,3]]=trace[[3]];
  m
]
(* Assumes that the original matrix has ordered 
   symbol names in diagonal, e.g. m11, m22, m33 *)


(* ::Subsubsection::Closed:: *)
(*quatToFromMatrix, matrix input*)


qFromNumberM[minput_]:=
Module[
  {m,\[CapitalDelta]Vectors,difforder,axisIndexA,axisIndexB,projRotAxMax,axis,angle},
  m=Orthogonalize@minput;
  \[CapitalDelta]Vectors=m-IdentityMatrix@3;
  difforder=Ordering@N@Diagonal@m;
  axisIndexA=First@difforder;
  axisIndexB=difforder[[2]];
  axis=Normalize[\[CapitalDelta]Vectors[[axisIndexA]]\[Cross]\[CapitalDelta]Vectors[[axisIndexB]]];
  projRotAxMax=Dot[m[[axisIndexA]],axis]*axis;
  angle=signedAngleBetweenVectors[
    UnitVector[3,axisIndexA]-projRotAxMax,m[[axisIndexA]]-projRotAxMax,axis
  ];
  Times[Sqrt@Norm@minput,Flatten@{Cos[angle/2],Sin[angle/2]*axis}]//qOut
]


qFromNumeric180\[Degree]M[minput_]:=
Module[
  {m,\[CapitalDelta]Vectors,difforder,axisIndexA,axisIndexB,axis},
  m=Orthogonalize@minput;
  \[CapitalDelta]Vectors=m-IdentityMatrix@3;
  difforder=Ordering@N@Diagonal@m;
  axisIndexA=First@difforder;
  axisIndexB=difforder[[2]];
  axis=Normalize[\[CapitalDelta]Vectors[[axisIndexA]]\[Cross]\[CapitalDelta]Vectors[[axisIndexB]]];
  Times[Sqrt@Norm@minput,Flatten@{0,axis}]//qOut
]


qFromNumericM[m_]:=Module[
  {cos\[Theta]h,sin\[Theta]h,axis,v1,v2,v3},
  If[Norm@N@m!=1,Return@qFromNumberM[N@m]];
  cos\[Theta]h=Sqrt[1+Tr@m]/2;
  sin\[Theta]h=Sqrt[3-Tr@m]/2;
  v1=m[[2,3]]-m[[3,2]];
  v2=m[[3,1]]-m[[1,3]];
  v3=m[[1,2]]-m[[2,1]];
  axis=sin\[Theta]h Normalize@{v1,v2,v3};
  {cos\[Theta]h,axis}//Flatten//Simplify//qOut
]


qFromSymbolicM[m_]:=Module[
  {q0,q1,q2,q3},
  If[is180\[Degree]Matrix[m],Print@Framed["Cannot convert matrix to quat",FrameStyle->Red];Return@m];
  q0=Sqrt[m[[1,1]]+m[[2,2]]+m[[3,3]]+1]/2;
  q1=(m[[2,3]]-m[[3,2]])/(4 q0);
  q2=(m[[3,1]]-m[[1,3]])/(4 q0);
  q3=(m[[1,2]]-m[[2,1]])/(4 q0);
  {q0,q1,q2,q3}//If[LeafCount[#]>220,Simplify@#,#]&//qOut
]


qFromSymbolicIdentityM[m_]:=With[
  {e=First@Diagonal[m]},
  {PowerExpand@Sqrt@e,0,0,0}//qOut
]


qFrom90\[Degree]AroundBaseAxisM[m_]:=Module[
  {e,i,sign},
  e=Last@Sort@Diagonal[m];
  i=First@Flatten@Position[Diagonal@m,e];
  sign=Switch[i,
    1,Sign@(m/e)[[2,3]],
    2,Sign@(m/e)[[3,1]],
    3,Sign@(m/e)[[1,2]]
  ];
  ReplacePart[{PowerExpand@Sqrt[e/2],0,0,0},i+1->sign*PowerExpand@Sqrt[e/2]]//qOut
]


qFrom120\[Degree]AroundOctantDiagonalM[m_]:=Module[
  {e,signs},
  e=Last@Sort@Flatten@m;
  signs={
    1,
    (m/e)[[2,3]]-(m/e)[[3,2]],
    (m/e)[[3,1]]-(m/e)[[1,3]],
    (m/e)[[1,2]]-(m/e)[[2,1]]
  };
  ConstantArray[PowerExpand@Sqrt@e/2,4]*signs//qOut
]


qFrom180\[Degree]AroundBaseAxisM[m_]:=Module[
  {e,i},
  e=Last@Sort@Diagonal[m];
  i=First@Flatten@Position[Diagonal@m,e,{1}];
  ReplacePart[{0,0,0,0},i+1->PowerExpand@Sqrt@e]//qOut
]


qFrom180\[Degree]AroundPlaneDiagonalM[m_]:=Module[
  {e,i,sign,q},
  e=-Last@Sort@Diagonal[m];
  i=First@Flatten@Position[Diagonal@m,-e];
  sign=Sign@First@Total@Cases[ReplacePart[m,{j_,j_}->0]//Flatten,Except[0|0.]];
  q=PowerExpand@ReplacePart[{0,Sqrt[e/2],Sqrt[e/2],Sqrt[e/2]},i+1->0];
  Replace[q,{head:Repeated[0],e1:Except[0],tail__}:>{head,sign*e1,tail}]//qOut
]


qFrom180\[Degree]AroundAxisInBasePlaneM[m_]:=Module[
  {axis,sign},
  axis=PowerExpand[Map[Sqrt,Diagonal@m,{2}]/.-s_Symbol^2:>0];
  sign=Cases[ReplacePart[m,{i_,i_}->Nothing],Except[0|0.],{2}]//First//First//Sign;
  Replace[Prepend[axis,0],{head:Repeated[0],e1:Except[0],tail__}:>{head,sign*e1,tail}]//qOut
]


qFrom180\[Degree]AroundAxisOutOfBasePlaneM[m_]:=Module[
  {axis,signs},
  axis=PowerExpand[Map[Sqrt,Diagonal@m,{2}]/.-s_Symbol^2:>0];
  signs=Sign@Map[First,Apply[Times,ReplacePart[m,{i_,i_}->1]]];
  Prepend[signs*axis,0]//qOut
]


qBasicFromM[m_]:=Module[
  {q0,q1,q2,q3},
  q0=Cases[Tr@m,Except[-_]]//Cases[#,_Symbol,-1]&//First;
  q1=DeleteCases[m[[2,3]]-m[[3,2]]//Expand,q0]/4;
  q2=DeleteCases[m[[3,1]]-m[[1,3]]//Expand,q0]/4;
  q3=DeleteCases[m[[1,2]]-m[[2,1]]//Expand,q0]/4;
  {q0,q1,q2,q3}//qOut
]


signedAngleBetweenVectors[startV_,endV_,axis_]:=Module[
  {sign,sinvalue,cosvalue,angle},
  sign[x_]:=Sign[x] /. 0->1;
  sinvalue=Cross[Normalize@startV,Normalize@endV]//Norm;
  cosvalue=Dot[Normalize@startV,Normalize@endV];
  angle=Which[
    sinvalue<=cosvalue,ArcSin@sinvalue,
    Abs@cosvalue<sinvalue,ArcCos@cosvalue,
    True,\[Pi]-ArcSin@sinvalue
  ];
  sign@Dot[Cross[startV,endV],axis]*angle
]


(* ::Subsubsection::Closed:: *)
(*quatToFromEulerZYX, quat input*)


eulerZYXangles[m_,singularity_]:=Module[
  {z,y,x,angles=ConstantArray[0,{3,3}]},
  Switch[singularity,
    False,
      y=-Sign[m[[1,3]]]*VectorAngle[{m[[1,1]],m[[1,2]],0},m[[1]]];
      z=signedAngleBetweenVectors[{1,0,0},{m[[1,1]],m[[1,2]],0},{0,0,1}];
      x=signedAngleBetweenVectors[{-Sin@z,Cos@z,0},m[[2]],m[[1]]],
    True,
      y=-ArcSin@m[[1,3]];
      z=signedAngleBetweenVectors[{0,1,0},m[[2]],{0,0,1}];
      x=0
  ];
  angles[[1]]={z,y,x}/\[Degree]//N//vOut;
  Switch[singularity,
    False,
      angles[[2]]=
      {
        angles[[1,1]]-180//Mod[#,360,-180]&,
        180-angles[[1,2]]//Mod[#,360,-180]&,
        angles[[1,3]]-180//Mod[#,360,-180]&
      }/.-180->180;
      Sort[Take[angles,2],Norm@#1<Norm@#2&],
    True,
      angles[[2,1]]=If[angles[[1,1]]!=0,0,180];
      angles[[2,2]]=angles[[1,2]];
      angles[[2,3]]=If[angles[[1,1]]!=0,-Sign@angles[[1,2]]*angles[[1,1]],180];
      angles[[3,1]]="t";
      angles[[3,2]]=angles[[1,2]];
      angles[[3,3]]=Which[
        Sign@angles[[1,2]]==-1,angles[[1,1]]-"t",
        Sign@angles[[1,1]]==-1,"t"+Abs@angles[[1,1]],
        True,"t"-angles[[1,1]]
      ];
      angles
  ]
]


(* ::Subsubsection::Closed:: *)
(*quatFromAlignedMatrix*)


alignedMatrixDialogs[]:=Module[
  {m=IdentityMatrix@3,row=1,col,ybtns={"X"->1,"Y"->2,"Z"->3},mDisplay,back},
  mDisplay[m_,highlight_]:=
    Grid[
      m,
      ItemSize->2,Alignment->Center,
      Background->{White,{highlight->Lighter[RGBColor[0.2,0.7,0.7],0.6]}},
      Frame->True
    ]
  ;
  While[row!=0,
    back=False;
    row=ChoiceDialog[mDisplay[m,0],{"X row"->1,"Y row"->2,"Finish"->0}];
    Switch[row,
      1,
        While[!back,
          col=ChoiceDialog[mDisplay[m,1],{"X"->1,"Y"->2,"Z"->3,"\[PlusMinus]"->4,"Back"->5},Appearance->"Horizontal"->{2,3}];
          Switch[col,
            5,back=True,
            4, m[[1]]=-m[[1]],
            _,m[[1]]=UnitVector[3,col]
          ];
          If[Abs@m[[2]]==Abs@m[[1]],m[[2]]=RotateRight[m[[1]]]/.-1->1];
          m[[3]]=m[[1]]\[Cross]m[[2]]
        ],
      2,
        While[!back,
          col=ChoiceDialog[mDisplay[m,2],Join[Pick[ybtns,m[[1]],0],{"\[PlusMinus]"->4,"Back"->5}],Appearance->"Horizontal"->{2,2}];
          Switch[col,
            5,back=True,
            4, m[[2]]=-m[[2]],
            _,m[[2]]=UnitVector[3,col]
          ];
          m[[3]]=m[[1]]\[Cross]m[[2]]
        ]
    ]
  ];
  m
]


(* ::Subsubsection::Closed:: *)
(*Rewriting quat*)


rewriteQuatType39[q_List]:=If[FreeQ[First@q,Sin[_]],rewriteQuatType39HA[q],rewriteQuatType39FA[q]]


rewriteQuatType39HA[q_List]:=Module[
  {type,q0,qV},
  type=If[MatchQ[First@q,1/2*Sqrt[_]],"A","B"];
  q0=First@Cases[q,Cos[_Symbol],-1]/.Cos[\[Theta]_]:>Cos[\[Theta]/2];
  qV=Switch[type,
    "A",Take[q,-3]/.num_/Sqrt[_]:>num/(2 q0),
    "B",Take[q,-3]/.num_/Sqrt[_]:>num/(2 Sqrt[2] q0)
  ]//TrigExpand;
  {q0,qV}//Flatten
]


rewriteQuatType39FA[q_List]:=Module[
  {q0,qV},
  q0=First@Cases[q,Cos[_],-1];
  qV=Take[q,-3]/.num_/Sqrt[_]:>num/(2 q0)//TrigExpand;
  {q0,qV}//Flatten
]


rewriteQuatType41[q_List]:=If[Depth[q]<8,rewriteQuatType41HA[q],rewriteQuatType41FA[q]]


rewriteQuatType41HA[q_List]:=Module[
  {eulerType,sign,angles,mid,\[Alpha]\[Beta]\[Gamma],q0,qV},
  (* Structure *)
  eulerType=If[Length@Cases[q,_Symbol,-1]<60,"IJK","IJI"];
  sign=If[Count[First@q,-_,-1]>0,"N","P"];
  angles=Cases[q,_Symbol,-1]//DeleteDuplicates//Sort;
  mid=Function[First@First@Position[#,Max@#]][Count[q,Cos[#],-1]&/@angles];
  \[Alpha]\[Beta]\[Gamma]=Switch[eulerType,
    "IJK",angles,
    "IJI",{angles[[mid]],angles[[Mod[mid+1,3,1]]],angles[[Mod[mid-1,3,1]]]}
  ];
  (* q0 *)
  q0=Switch[{eulerType,sign},
    {"IJK","N"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[2]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[3]]/2]-Sin[\[Alpha]\[Beta]\[Gamma][[1]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[2]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[3]]/2],
    {"IJK","P"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[2]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[3]]/2]+Sin[\[Alpha]\[Beta]\[Gamma][[1]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[2]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[3]]/2],
    {"IJI","N"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[2]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[3]]/2]-Cos[\[Alpha]\[Beta]\[Gamma][[1]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[2]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[3]]/2],
    {"IJI","P"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[2]]/2] Cos[\[Alpha]\[Beta]\[Gamma][[3]]/2]+Cos[\[Alpha]\[Beta]\[Gamma][[1]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[2]]/2] Sin[\[Alpha]\[Beta]\[Gamma][[3]]/2]
  ];
  (* qV *)
  qV=Take[q,-3]/.num_/Sqrt[_]:>num/(2 q0)//TrigExpand;
  (* Out *)
  {q0,qV}//Flatten
]


rewriteQuatType41FA[q_List]:=Module[
  {eulerType,sign,angles,mid,\[Alpha]\[Beta]\[Gamma],q0,qV},
  (* Structure *)
  eulerType=If[Length@Cases[First@q,_Symbol,-1]>10,"IJK","IJI"];
  sign=Switch[eulerType,
    "IJK",If[Count[First@q,Cos[_],-1]>7,"N","P"],
    "IJI",If[Count[First@q,-_,-1]==0,"N","P"]
  ];
  angles=Cases[q,_Symbol,-1]//DeleteDuplicates//Sort;
  mid=Function[First@First@Position[#,Max@#]][Count[q,Cos[#],-1]&/@angles];
  \[Alpha]\[Beta]\[Gamma]=Switch[eulerType,
    "IJK",angles,
    "IJI",{angles[[mid]],angles[[Mod[mid+1,3,1]]],angles[[Mod[mid-1,3,1]]]}
  ];
  (* q0 *)
  q0=Switch[{eulerType,sign},
    {"IJK","N"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]] Cos[\[Alpha]\[Beta]\[Gamma][[2]]] Cos[\[Alpha]\[Beta]\[Gamma][[3]]]-Sin[\[Alpha]\[Beta]\[Gamma][[1]]] Sin[\[Alpha]\[Beta]\[Gamma][[2]]] Sin[\[Alpha]\[Beta]\[Gamma][[3]]],
    {"IJK","P"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]] Cos[\[Alpha]\[Beta]\[Gamma][[2]]] Cos[\[Alpha]\[Beta]\[Gamma][[3]]]+Sin[\[Alpha]\[Beta]\[Gamma][[1]]] Sin[\[Alpha]\[Beta]\[Gamma][[2]]] Sin[\[Alpha]\[Beta]\[Gamma][[3]]],
    {"IJI","N"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]] Cos[\[Alpha]\[Beta]\[Gamma][[2]]] Cos[\[Alpha]\[Beta]\[Gamma][[3]]]-Cos[\[Alpha]\[Beta]\[Gamma][[1]]] Sin[\[Alpha]\[Beta]\[Gamma][[2]]] Sin[\[Alpha]\[Beta]\[Gamma][[3]]],
    {"IJI","P"},Cos[\[Alpha]\[Beta]\[Gamma][[1]]] Cos[\[Alpha]\[Beta]\[Gamma][[2]]] Cos[\[Alpha]\[Beta]\[Gamma][[3]]]+Cos[\[Alpha]\[Beta]\[Gamma][[1]]] Sin[\[Alpha]\[Beta]\[Gamma][[2]]] Sin[\[Alpha]\[Beta]\[Gamma][[3]]]
  ];
  (* qV *)
  qV=Switch[eulerType,
    "IJK",Take[q,-3]/.num_/Sqrt[_]:>num/(2 q0)//TrigExpand,
    "IJI",Take[q,-3]/.num_/Sqrt[_]:>num/q0//TrigExpand
  ];
  (* Out *)
  {q0,qV}//Flatten
]


rewriteQuatType42[q_List]:=Module[
  {cosines,q0,qV},
  cosines=Cos[First@#/2]&/@Take[Cases[q,Cos[_],-1],2];
  q0=Times@@cosines;
  qV=Take[q,-3]/.num_/Sqrt[_]:>num/(2 q0)//TrigExpand;
  {q0,qV}//Flatten
]


rewriteQuatType49[q_List]:=If[FreeQ[First@q,Sin[_]],rewriteQuatType49HA[q],rewriteQuatType49FA[q]]


rewriteQuatType49HA[q_List]:=Module[
  {type,q0,qV},
  type=If[MatchQ[First@q,Sqrt[_]/(2 Sqrt[2])],"A","B"];
  q0=First@Cases[q,Cos[_Symbol],-1]/.Cos[\[Theta]_]:>Cos[\[Theta]/2];
  qV=Switch[type,
    "A",Take[q,-3]/.num_/Sqrt[_]:>num/(2 Sqrt[2]q0),
    "B",Take[q,-3]/.num_/Sqrt[_]:>num/(4 q0)
  ]//TrigExpand;
  {q0,qV}//Flatten
]


rewriteQuatType49FA[q_List]:=Module[
  {q0,qV},
  q0=First@Cases[q,Cos[_],-1];
  qV=Take[q,-3]/.num_/Sqrt[_]:>num/(2 q0)//TrigExpand;
  {q0,qV}//Flatten
]


(* ::Subsubsection::Closed:: *)
(*Output formatting*)


qOut[qIn_List]:=With[
  {q=roundNumbers[qIn]},
  quat@@Switch[qType[quat@@q],
    1,If[First@q<0,-q,q],
    2,If[Chop@N@First@q<0,-q,q]/.{
        1/2 Sqrt[1-Cos[\[Alpha]_]]:>Sin[\[Alpha]/2]/Sqrt[2],
        1/2 Sqrt[1+3 Cos[\[Alpha]_]^2-Sin[\[Alpha]_]^2]:>Cos[\[Alpha]],1/2 Sqrt[r_ (3-3 Cos[\[Alpha]_]^2+Sin[\[Alpha]_]^2)]:>Sqrt[r] Sin[\[Alpha]],
        Sqrt[Times[r_/;r===1/2,(1+Cos[\[Alpha]_]^2-Sin[\[Alpha]_]^2)]]:>Cos[\[Alpha]],Sqrt[r_ (1-Cos[\[Alpha]_]^2+Sin[\[Alpha]_]^2)]:>Sqrt[2 r] Sin[\[Alpha]],
        1/2 Sqrt[1-Cos[\[Alpha]_]^2+Sin[\[Alpha]_]^2]:>Sqrt[1/2] Sin[\[Alpha]]
      },
    3|4,q/.{
        1/2 Sqrt[2+2*Cos[\[Theta]_]]:>Cos[\[Theta]/2],Sin[\[Theta]_]/Sqrt[2+2*Cos[\[Theta]_]]:>Sin[\[Theta]/2],
        (2*Cos[\[Theta]_]*Sin[\[Theta]_]+Sin[2*\[Theta]_])/(2*Sqrt[2+2*Cos[2*\[Theta]_]]):>Sin[\[Theta]],
        1/2 Sqrt[1+3 Cos[\[Theta]_]^2-x_^2*Sin[\[Theta]_]^2]:>Cos[\[Theta]],
        1/2 Sqrt[1+3 Cos[\[Theta]_]^2-x_^2*Sin[\[Theta]_]^2-y_^2*Sin[\[Theta]_]^2]:>Cos[\[Theta]],
        1/2 Sqrt[1+3 Cos[\[Theta]_]^2-x_^2*Sin[\[Theta]_]^2-y_^2*Sin[\[Theta]_]^2-z_^2*Sin[\[Theta]_]^2]:>Cos[\[Theta]],
        x_*Sin[a_]/Sqrt[1+3 Cos[\[Theta]_]^2-x_^2*Sin[\[Theta]_]^2]:>x*Sin[\[Theta]],
        x_*Sin[a_]/Sqrt[1+3 Cos[\[Theta]_]^2-x_^2*Sin[\[Theta]_]^2-y_^2*Sin[\[Theta]_]^2]:>x*Sin[\[Theta]]
      },
    39,rewriteQuatType39[q],
    41,rewriteQuatType41[q],
    42,rewriteQuatType42[q],
    49,rewriteQuatType49[q],
    _,q
  ]
]


mOut[m_]:=m//Simplify//roundNumbers[#]&


vOut[v_List]:=v//Simplify//roundNumbers[#]&


\[Theta]VOut[rotIn_List]:=With[
  {rot=roundNumbers[rotIn]},
  Simplify[PowerExpand[rot]/. 1/Sqrt[1-Cos[\[Theta]_]^2]:>1/Sin[\[Theta]]]
]


roundNumbers[x_]:=ReplaceAll[x,n_?MachineNumberQ:>If[Abs[n-Round@n]<10^-12,Round@n,n]]


(* ::Section::Closed:: *)
(*End*)


End[]


SyntaxInformation[quat]={"ArgumentsPattern"->{_,_,_,_}};


EndPackage[]
