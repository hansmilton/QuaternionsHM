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
(*In contrast, Mathematica's inbuilt function RotationMatrix uses the active convention.*)
(*The matrix rows are the base axes of the reference frame, as seen from a rotated frame.*)


(* ::Text:: *)
(*The passive convention matches the result of mapping the function quatRotateVector over*)
(*the rows of an identity 3x3 matrix.*)


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
(**)
(*After May 2022 several maintenance updates*)


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
"Converts a rotation from quaternion to angle-axis representation, or vice verse. The axis
of rotation is a 3D vector. As input it can have any length. As output it has unit length.
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
"Converts a quaternion to a 3x3 matrix, or vice verse.
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


quatVersionDate::usage="Date of last update";


(* ::Section::Closed:: *)
(*Begin Private*)


Begin["`Private`"]


(* ::Subsection::Closed:: *)
(*Upvalues for quat*)


quat/:NonCommutativeMultiply[quat[p0_,p1_,p2_,p3_]?quatQ,quat[q0_,q1_,q2_,q3_]?quatQ]:=
Module[
  {pV,qV,r0,rV},
  pV={p1,p2,p3}; qV={q1,q2,q3};
  r0=p0 q0-Dot[pV,qV];
  rV=p0 qV+q0 pV+Cross[pV,qV];
  Prepend[rV,r0]//qOut
]


quat/:Power[base_quat?quatQ,-1]/;Nor[Norm@base===0,Norm@base===0.]:=
  Conjugate[base]/Norm[base]^2/.sqAbsRule//roundNumbers
quat/:Power[base_quat?quatQ,exponent_?scalarQ]/;Nor[Norm@base===0,Norm@base===0.]:=
  Exp[exponent*Log@base]


quat/:Conjugate[quat[q0_,q1_,q2_,q3_]?quatQ]:=quat[q0,-q1,-q2,-q3]


quat/:Norm[q_quat?quatQ]:=Norm[List@@q]/.sqAbsRule


quat/:Normalize[q_quat?quatQ]:=Normalize[List@@q]//qOut


quat/:Exp[q_quat?quatQ]:=With[
  {q0=First@q,qV=Rest[List@@q]},
  E^q0 {Cos[Norm@qV],Normalize[qV] Sin[Norm@qV]}/.sqAbsRule//Flatten//qOut
]


quat/:Log[q_quat?quatQ]/;Nor[Norm@q===0,Norm@q===0.]:=With[
  {q0=First@q,qV=Rest[List@@q]},
  {Log@TrigFactor@Norm[q], (Normalize[qV]/.sqAbsRule)*ArcCos[q0/Norm[q]]}//Flatten//quat@@#&//roundNumbers
]


quat/:Times[s_?scalarQ,q_quat?quatQ]:=s*#&/@q


quat/:Plus[s_?scalarQ,q_quat?quatQ]:=s+#&/@q
quat/:Plus[p_quat?quatQ,q_quat?quatQ]:=Apply[quat,List@@p+List@@q]


quat/:Times[NonCommutativeMultiply[p_quat,1],q_quat]:=p**q


(* ::Subsection::Closed:: *)
(*Public functions*)


quatToFromList[q_?quatQ]:=List@@q
quatToFromList[l_?qlistQ]:=l//qOut


quatToFrom\[Theta]V[q_?quatQ]/;Norm@Chop@Rest[List@@q]===0:={First[q],{0,0,0}}
quatToFrom\[Theta]V[q:quat[q0_,__]?quatQ]/;And[NumericQ@q0,symvectQ[Rest[List@@q]],Abs[q0]>=1]:=
  {0,{0,0,0}}
quatToFrom\[Theta]V[q_?quatQ]/;symvectQ[Rest[List@@q]]:=Module[
  {s=2 ArcCos[First@q],v=Rest[List@@q]},
  v=If[Count[Chop@v,0]==2,Replace[v,_Symbol->1,{1,2}],v/Sqrt[1-First[q]^2]];
  {s,v}
]//\[Theta]VOut 
quatToFrom\[Theta]V[q_?quatQ]:=
  {2 ArcCos[First@q/Norm@q],Normalize@Take[List@@q,-3]}//\[Theta]VOut

quatToFrom\[Theta]V[PatternSequence[\[Theta]_?scalarQ,v_?vectQ]|{\[Theta]_?scalarQ,v_?vectQ}]/;Norm@Chop@v===0:=
  quat[\[Theta],0,0,0]
quatToFrom\[Theta]V[PatternSequence[\[Theta]_?scalarQ,v_?symvectQ]|{\[Theta]_?scalarQ,v_?symvectQ}]:=
  If[Count[Chop@v,0]==2,
    {Cos[\[Theta]/2],Sin[\[Theta]/2] v/.r_Symbol*Sin[\[Theta]/2]:>Sin[\[Theta]/2]},{Cos[\[Theta]/2],Sin[\[Theta]/2] v}
  ]//Flatten//qOut
quatToFrom\[Theta]V[PatternSequence[\[Theta]_?scalarQ,v_?vectQ]|{\[Theta]_?scalarQ,v_?vectQ}]:=
  {Cos[\[Theta]/2],Sin[\[Theta]/2] Normalize@v}//Flatten//qOut


quatToFromMatrix[q_?quatQ/;qType@q==40]:=mFromSymbolsOnlyQ[q]
quatToFromMatrix[q_?quatQ]:=Map[quatRotateVector[q,#]&,IdentityMatrix[3]]//mOut

quatToFromMatrix[m_?matQ]:=Switch[mType@m,
  1,qFromNumberM[m],
  2,qFromNumericM[m],
  3,qFromSymbolicM[m],
  4,qFromSymbolicM[m],
  21,qFromNumeric180\[Degree]M[m],
  30,qFromSymbolicAngleRotAroundBaseAxisM[m],
  31,qFromSymbolicIdentityM[m],
  32,qFrom90\[Degree]AroundBaseAxisSymbolicM[m],
  33,qFrom120\[Degree]AroundOctantDiagonalSymbolicM[m],
  34,qFrom180\[Degree]AroundBaseAxisSymbolicM[m],
  35,qFrom180\[Degree]AroundPlaneDiagonalSymbolicM[m],
  36,qFrom180\[Degree]AroundAxisInBasePlaneSymbolicM[m],
  38,qFromEuler2SymbolicAnglesM[m],
  41,qFromSymbolsOnlyM[m],
  45,qFrom180\[Degree]AroundAxisOutOfBasePlaneSymbolicM[m],
  46,qFromSymbolicMRotationInBasePlane[m],
  47,qFromSymbolicMRotationOutOfBasePlanes[m],
  48,qFromEuler3SymbolicAnglesDistinctAxesM[m],
  49,qFromEuler3SymbolicAnglesRepeatedAxisM[m],
  _,m
]


quatRotateVector[quat[q0_,q1_,q2_,q3_]?quatQ,v_?vectQ]:=With[
  {qV={q1,q2,q3}},
  (q0^2-q1^2-q2^2-q3^2)*v+2*Dot[qV,v]*qV+2*q0*Cross[qV,v]//roundNumbers
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


quatFromAlignedMatrix[]:=N@quatToFromMatrix[alignedMatrixDialogs[]]/.{0.->0,1.->1,-1.->1}


(* ::Subsection::Closed:: *)
(*Lower level functions*)


(* ::Subsubsection::Closed:: *)
(*Checking arguments to public functions*)


scalarQ[e_]:=And[FreeQ[e,Complex],ReplaceAll[e,s_Symbol/;Context@s=!="System`":>RandomReal[]]//NumericQ]


qlistQ[l_]:=VectorQ[l,scalarQ]&&Length[l]==4


vectQ[v_]:=VectorQ[v,scalarQ]&&Length[v]==3


quatQ[q_]:=MatchQ[q,quat[_?scalarQ,_?scalarQ,_?scalarQ,_?scalarQ]]


matQ[m_]:=MatrixQ[m,scalarQ]&&Dimensions[m]=={3,3}


symvectQ[v_]:=And[vectQ[v],Count[Chop@v,0|(e_/;!NumericQ[e])]==3,Count[Chop@v,0]<3]


(* ::Subsubsection::Closed:: *)
(*Type codes*)


(* ::Text:: *)
(*    0:  Nonvalid*)
(*    1:  All explicit numbers*)
(*    2:  All numeric*)
(*    21: Numeric 180\[Degree] rotation matrix*)
(*    3:  Mixed numeric and symbolic*)
(*    30: Matrix representing symbolic angle rotation around a base axis*)
(*    31: Symbolic identity matrix*)
(*    32: Symbolic matrix representing 90\[Degree] rotation around a base axis*)
(*    33: Symbolic matrix representing 120\[Degree] rotation around an octant diagonal*)
(*    34: Symbolic matrix representing 180\[Degree] rotation around a base axis*)
(*    35: Symbolic matrix representing 180\[Degree] rotation around a base plane diagonal*)
(*    36: Symbolic matrix representing 180\[Degree] rotation around axis in a base plane*)
(*    38: Euler 2 symbolic angles matrix*)
(*    4:  All symbolic*)
(*    40: Quat from matrix with only symbols*)
(*    41: Matrix from quat with only symbols (that can be signed)*)
(*    45: Symbolic matrix representing 180\[Degree] rotation around axis out of base planes*)
(*    46: Symbolic matrix representing rotation around axis in a base plane*)
(*    47: Symbolic matrix representing rotation around axis out of base planes*)
(*    48: Euler 3 symbolic angles matrix, distinct axes*)
(*    49: Euler 3 symbolic angles matrix, repeated axis*)


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
    1|2|3,type,
    4,Which[
      isQuatType40[q],40,
      True,4
    ]
  ];
  type
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
        isMatrixType30[m],30,
        isMatrixType31[m],31,
        isMatrixType32[m],32,
        isMatrixType33[m],33,
        isMatrixType34[m],34,
        isMatrixType35[m],35,
        isMatrixType36[m],36,
        isMatrixType38[m],38,
        True,3
      ],
    4,Which[
        isInvalidSymbolicMatrix[m],0,
        isMatrixType41[m],41,
        isMatrixType45[m],45,
        isMatrixType46[m],46,
        isMatrixType47[m],47,
        isMatrixType48[m],48,
        isMatrixType49[m],49,
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
  Count[m,0|0.,{2}]==6,
  Count[m,_Symbol,{-1}]==3,
  SameQ@@Abs@Cases[m,Except[0|0.],{2}]
]


isLHAlignedMatrix[m_]:=With[
  {res=And[isAlignedMatrix[m],Det[Replace[m,_Symbol->1,{-1}]]<1]},
  If[res,Print@Framed["Lefthanded aligned matrix",FrameStyle->Red]];
  res
]


isMatrixType30[m_]:=With[
  {
    ms=Simplify@m,
    norm=FirstCase[Simplify[m],_?Positive,1,{2}],
    diag={1,Cos[\[Alpha]_Symbol],Cos[\[Alpha]_Symbol]},
    offdiags={-Sin[\[Alpha]_Symbol],Sin[\[Alpha]_Symbol]}
  },
  And[
    Count[m,0|0.,{2}]==4,
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==1,
    Cases[ms,Except[0|0.,_?NumberQ],Infinity]//Abs//Apply[SameQ],
    MatchQ[Sort@Diagonal@ms,norm*diag],
    MatchQ[ReplacePart[ms,{i_,i_}->Nothing]//Flatten//DeleteCases[0|0.]//Sort,norm*offdiags],
    DisjointQ[First@Position[Diagonal@ms,_?NumberQ,{1}],Union@@Position[ms,Alternatives@@(norm*offdiags),{2}]]
  ]
]


isMatrixType31[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal[m],Except[0|0.]]==3,
  Count[Map[FreeQ[#,_?Negative]&,Diagonal@m],True]==3
]


isMatrixType32[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,0|0.]==2,
  FreeQ[FirstCase[Diagonal@m,Except[0|0.]],_?Negative],
  UnsameQ@@Cases[ReplacePart[m,{i_,i_}->Nothing],Except[0|0.],{2}]
]


isMatrixType33[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,0|0.]==3,
  Count[Map[FreeQ[#,_?Negative]&,Cases[m,Except[0|0.],{2}]],True]//MatchQ[1|3]
]


isMatrixType34[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,Except[0|0.]]==3,
  Count[Map[FreeQ[#,_?Negative]&,Diagonal@m],True]==1
]


isMatrixType35[m_]:=And[
  isAlignedMatrix[m],
  Count[Diagonal@m,0|0.]==2,
  !FreeQ[FirstCase[Diagonal@m,Except[0|0.]],_?Negative],
  SameQ@@Cases[ReplacePart[m,{i_,i_}->Nothing],Except[0|0.],{2}]
]


isMatrixType36[m_]:=With[
  {
    mx=Expand@m,
    norm=Function[If[And@@
      Through[{MatchQ[{_?Negative,_?Negative,_?Positive}],SameQ@*Abs}[Sort@Cases[Factor@Diagonal@#,_?NumberQ,{2}]]],
      FirstCase[Diagonal@#,_?Positive,"",{3}],1
    ]],
    diags=Alternatives[
      {{-s_Symbol^2,-t_Symbol^2},{s_Symbol^2,-t_Symbol^2},{-s_Symbol^2,t_Symbol^2}},
      {{s_Symbol^2,-t_Symbol^2},{-s_Symbol^2,-t_Symbol^2},{-s_Symbol^2,t_Symbol^2}},
      {{s_Symbol^2,-t_Symbol^2},{-s_Symbol^2,t_Symbol^2},{-s_Symbol^2,-t_Symbol^2}}
    ],
    offdiags=Alternatives[{2*s_Symbol*t_Symbol,2*s_Symbol*t_Symbol},{-2*s_Symbol*t_Symbol,-2*s_Symbol*t_Symbol}]
  },
  And[
    is180\[Degree]Matrix[m],
    Count[m,0|0.,{2}]==4,
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==2,
    MatchQ[Diagonal@mx/norm[mx]//Expand//MapApply[List],diags],
    MatchQ[Flatten@ReplacePart[mx,{i_,i_}->0]/norm[mx]//DeleteCases[0|0.],offdiags],
    DisjointQ[FirstPosition[Diagonal@mx/norm[mx]//Expand//MapApply[List],diags[[1,1]]],Position[mx/norm[mx],offdiags[[All,1]]]//Flatten]  
  ]
]


isMatrixType38[m_]:=With[
  {offdiag=
    {
      0,
      Sin[s_Symbol],
      Cos[t_Symbol]*Sin[s_Symbol],
      Sin[t_Symbol],
      Cos[s_Symbol]*Sin[t_Symbol],
      Sin[s_Symbol]*Sin[t_Symbol]
    }
  },
  And[
    Count[m,0|0.,{2}]==1,
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==2,
    MatchQ[Sort@Diagonal@m,{Cos[s_Symbol],Cos[t_Symbol],Cos[s_Symbol]*Cos[t_Symbol]}],
    MatchQ[DeleteCases[Sort@Flatten@ReplacePart[m,{i_,i_}->Nothing],-1,Infinity],offdiag],
    MatchQ[Count[Flatten@ReplacePart[m,{i_,i_}->Nothing],-1,Infinity],2|3]
  ]
]


isInvalidSymbolicMatrix[m_]:=Module[
  {chkA,chkB=False},
  chkA=Nand[DuplicateFreeQ@m,DuplicateFreeQ@Transpose@m];
  If[!chkA&&MatchQ[m,ConstantArray[_Symbol,{3,3}]],
    chkB=!OrderedQ[Diagonal@m,Order[#1,#2]&]
  ];
  If[chkA,Print@Framed["Matrix has duplicated rows or columns",FrameStyle->Red]];
  If[chkB,Print@Framed["Symbols in matrix diagonal are not in order",FrameStyle->Red]];
  Or[chkA,chkB]
]


isMatrixType41[m_]:=With[
  {offdiags=ReplacePart[m,{i_,i_}->Nothing]//Expand//Flatten},
  And[
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==4,
    MatchQ[Diagonal[m],Map[MapAt[Minus,a_Symbol^2-b_Symbol^2-c_Symbol^2-d_Symbol^2,#]&,{2,3,4}]],
    DuplicateFreeQ[offdiags],
    And@@Map[MatchQ[#,(-2|2)*p_Symbol*q_Symbol+(-2|2)*r_Symbol*s_Symbol]&,offdiags],
    And@@Map[DuplicateFreeQ[Cases[#,_Symbol,Infinity]]&,offdiags],
    Count[offdiags,Times[2,p_,q_]+Times[-2,r_,s_]]==3
  ]
]


isMatrixType45[m_]:=With[
  {
    mx=Expand@m,
    norm=Function[If[And@@
      Through[{MatchQ[{_?Positive,_?Negative,_?Negative}],SameQ@*Abs}[Cases[Factor@Diagonal@#,_?NumberQ,{2}]]],
      FirstCase[Diagonal@#,_?Positive,"",{3}],1
    ]],
    diag=Map[MapAt[Minus,-x_Symbol^2-y_Symbol^2-z_Symbol^2,#]&,{1,2,3}],
    offdiags={x_Symbol*y_Symbol|-x_Symbol*y_Symbol,x_Symbol*z_Symbol|-x_Symbol*z_Symbol,y_Symbol*z_Symbol|-y_Symbol*z_Symbol}
  },
  And[
    is180\[Degree]Matrix[m],
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==3,
    MatchQ[Diagonal@mx/norm[mx]//Expand,diag],
    MatchQ[Extract[m,{{1,2},{1,3},{3,2}}]/(2*norm[mx]),offdiags],
    Count[{m[[1,2]],m[[1,3]],m[[2,3]]},_?Negative,{2}]//MatchQ[0|2],
    Extract[m,{{1,2},{1,3},{2,3}}]===Extract[m,{{2,1},{3,1},{3,2}}]
  ]
]


isMatrixType46[m_]:=With[
  {
    mx=Expand@m,
    norm=Function[If[
      MatchQ[Diagonal@Factor@#,{n_?Positive*_,n_?Positive*_,n_?Positive*_}],FirstCase[Diagonal@Factor@#,n_?Positive,"",{2}],1
    ]],
    diags=Alternatives[
      Map[MapAt[Minus,{Cos[\[Alpha]_Symbol/2]^2,s_Symbol^2*Sin[\[Alpha]_Symbol/2]^2,t_Symbol^2*Sin[\[Alpha]_Symbol/2]^2},#]&,{{{2},{3}},3,2}],
      Map[MapAt[Minus,{Cos[\[Alpha]_Symbol/2]^2,s_Symbol^2*Sin[\[Alpha]_Symbol/2]^2,t_Symbol^2*Sin[\[Alpha]_Symbol/2]^2},#]&,{3,{{2},{3}},2}],
      Map[MapAt[Minus,{Cos[\[Alpha]_Symbol/2]^2,s_Symbol^2*Sin[\[Alpha]_Symbol/2]^2,t_Symbol^2*Sin[\[Alpha]_Symbol/2]^2},#]&,{3,2,{{2},{3}}}]
    ],
    offdiags=Alternatives[
      s_Symbol*Cos[\[Alpha]_Symbol/2]*Sin[\[Alpha]_Symbol/2],-s_Symbol*Cos[\[Alpha]_Symbol/2]*Sin[\[Alpha]_Symbol/2],
      t_Symbol*Cos[\[Alpha]_Symbol/2]*Sin[\[Alpha]_Symbol/2],-t_Symbol*Cos[\[Alpha]_Symbol/2]*Sin[\[Alpha]_Symbol/2],
      s_Symbol*t_Symbol*Sin[\[Alpha]_Symbol/2]^2,-s_Symbol*t_Symbol*Sin[\[Alpha]_Symbol/2]^2
    ]
  },
  And[
    Length[Cases[mx,_Symbol,{-1}]//DeleteDuplicates]==3,
    MatchQ[Diagonal@Factor@mx/norm[mx]//MapApply[List],diags],
    And@@Map[MatchQ[#/(2*norm[mx]),offdiags]&,Flatten@ReplacePart[mx,{i_,i_}->Nothing]],
    Length[ReplacePart[m,{i_,i_}->Nothing]//Flatten//DeleteDuplicates]==5,
    SameQ@@Cases[Flatten@ReplacePart[m,{i_,i_}->Nothing],x_/;FreeQ[x,Cos]],
    DisjointQ[
      FirstPosition[Diagonal@Factor@mx/norm[mx]//MapApply[List],diags[[1,1]]],Position[mx/(2*norm[mx]),Take[offdiags,-2]]//Flatten
    ]
  ]
]


isMatrixType47[m_]:=Module[
  {
    mx=Expand@m,
    diag=Map[MapAt[Minus,Cos[\[Alpha]_Symbol/2]^2-x_Symbol^2 Sin[\[Alpha]_Symbol/2]^2-y_Symbol^2 Sin[\[Alpha]_Symbol/2]^2-z_Symbol^2 Sin[\[Alpha]_Symbol/2]^2,#]&,{2,3,4}],
    norm=Function[FirstCase[Diagonal@Factor@#,_?Positive,1,{2}]],
    offdiags=Alternatives@@Plus@@@Map[{p_Symbol*Cos[\[Alpha]_Symbol/2]*Sin[\[Alpha]_Symbol/2],q_Symbol*r_Symbol*Sin[\[Alpha]_Symbol/2]^2}*#&,Tuples[{1,-1},2]]
   },
  And[
    Length[Cases[mx,_Symbol,{-1}]//DeleteDuplicates]==4,
    MatchQ[Diagonal@mx//Factor,diag|n_?Positive*diag//Factor],
    And@@Map[MatchQ[#/(2*norm[mx])//Expand,offdiags]&,Flatten@ReplacePart[mx,{i_,i_}->Nothing]],
    DuplicateFreeQ[Flatten@ReplacePart[mx,{i_,i_}->Nothing]]
  ]
]


isMatrixType48[m_]:=With[
  {
    diag1=
    {
      Cos[s_Symbol]*Cos[t_Symbol],Cos[t_Symbol]*Cos[u_Symbol],
      Cos[s_Symbol]*Cos[u_Symbol]+Sin[s_Symbol]*Sin[t_Symbol]*Sin[u_Symbol]
    },
    diag2=
    {
      Cos[s_Symbol]*Cos[t_Symbol],Cos[t_Symbol]*Cos[u_Symbol],
      Cos[s_Symbol]*Cos[u_Symbol]-Sin[s_Symbol]*Sin[t_Symbol]*Sin[u_Symbol]
    },
    offdiag=
    {
      Cos[t_Symbol]*Sin[s_Symbol],Sin[t_Symbol],Cos[t_Symbol]*Sin[u_Symbol],
      Cos[u_Symbol]*Sin[s_Symbol]*Sin[t_Symbol]+Cos[s_Symbol]*Sin[u_Symbol],
      Cos[s_Symbol]*Cos[u_Symbol]*Sin[t_Symbol]+Sin[s_Symbol]*Sin[u_Symbol],
      Cos[u_Symbol]*Sin[s_Symbol]+Cos[s_Symbol]*Sin[t_Symbol]*Sin[u_Symbol]
    }
  },
  And[
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==3,
    MatchQ[Sort@Diagonal@m,diag1|diag2],
    MatchQ[DeleteCases[Sort@Flatten@ReplacePart[m,{i_,i_}->Nothing],-1,Infinity],offdiag],
    MatchQ[Count[Flatten@ReplacePart[m,{i_,i_}->Nothing],-1,Infinity],3|5]
  ]
]


isMatrixType49[m_]:=With[
  {
    diag1=
    {
      Cos[t_Symbol],
      Cos[s_Symbol]*Cos[t_Symbol]*Cos[u_Symbol]+Sin[s_Symbol]*Sin[u_Symbol],
      Cos[s_Symbol]*Cos[u_Symbol]+Cos[t_Symbol]*Sin[s_Symbol]*Sin[u_Symbol]
    },
    diag2=
    {
      Cos[t_Symbol],
      Cos[s_Symbol]*Cos[t_Symbol]*Cos[u_Symbol]-Sin[s_Symbol]*Sin[u_Symbol],
      Cos[s_Symbol]*Cos[u_Symbol]-Cos[t_Symbol]*Sin[s_Symbol]*Sin[u_Symbol]
    },
    offdiag=
    {
      Cos[s_Symbol]*Sin[t_Symbol],Cos[u_Symbol]*Sin[t_Symbol],
      Sin[s_Symbol]*Sin[t_Symbol],Sin[t_Symbol]*Sin[u_Symbol],
      Cos[t_Symbol]*Cos[u_Symbol]*Sin[s_Symbol]+Cos[s_Symbol]*Sin[u_Symbol],
      Cos[u_Symbol]*Sin[s_Symbol]+Cos[s_Symbol]*Cos[t_Symbol]*Sin[u_Symbol]
    }
  },
  And[
    Length[Cases[m,_Symbol,{-1}]//DeleteDuplicates]==3,
    MatchQ[Sort@Diagonal@m,diag1|diag2],
    MatchQ[DeleteCases[Sort@Flatten@ReplacePart[m,{i_,i_}->Nothing],-1,Infinity],offdiag],
    MatchQ[Count[Flatten@ReplacePart[m,{i_,i_}->Nothing],-1,Infinity],3|4|5]
  ]
]


(* ::Subsubsection::Closed:: *)
(*quatToFromMatrix, quat input*)


mFromSymbolsOnlyQ[q:quat[q0_,q1_,q2_,q3_]]:=Module[
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
  {m,diffvects,changeorder,indexA,indexB,unitRotAx,scaledRotAx,angle},
  m=Orthogonalize@minput;
  diffvects=m-IdentityMatrix@3;
  changeorder=Ordering@N@Diagonal@m;
  indexA=changeorder[[1]];
  indexB=changeorder[[2]];
  unitRotAx=Normalize[diffvects[[indexA]]\[Cross]diffvects[[indexB]]];
  scaledRotAx=Dot[m[[indexA]],unitRotAx]*unitRotAx;
  angle=signedAngleBetweenVectors[
    UnitVector[3,indexA]-scaledRotAx,m[[indexA]]-scaledRotAx,unitRotAx
  ];
  Times[Sqrt@Norm@minput,Flatten@{Cos[angle/2],Sin[angle/2]*unitRotAx}]//qOut
]


qFromNumeric180\[Degree]M[minput_]:=
Module[
  {m,diffvects,changeorder,indexA,indexB,rotAxis},
  m=Orthogonalize@minput;
  diffvects=m-IdentityMatrix@3;
  changeorder=Ordering@N@Diagonal@m;
  indexA=changeorder[[1]];
  indexB=changeorder[[2]];
  rotAxis=Normalize[diffvects[[indexA]]\[Cross]diffvects[[indexB]]];
  Times[Sqrt@Norm@minput,Flatten@{0,rotAxis}]//qOut
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
  If[is180\[Degree]Matrix[m],Print@Framed["No quat matches this matrix",FrameStyle->Red];Return@m];
  q0=Sqrt[m[[1,1]]+m[[2,2]]+m[[3,3]]+1]/2;
  q1=(m[[2,3]]-m[[3,2]])/(4 q0);
  q2=(m[[3,1]]-m[[1,3]])/(4 q0);
  q3=(m[[1,2]]-m[[2,1]])/(4 q0);
  {q0,q1,q2,q3}//If[LeafCount[#]>220,Simplify@#,#]&//qOut
]


qFromSymbolicAngleRotAroundBaseAxisM[m_]:=Module[
  {norm,q0,qV,sign},
  norm=FirstCase[m,_?Positive,1,{2}];
  q0=Cos[FirstCase[m,_Symbol,"",{-1}]/2];
  qV=Diagonal[m]/norm/.{1->Sin[FirstCase[m,_Symbol,"",{-1}]/2],Cos[_]->0};
  sign=Switch[First@Position[Diagonal@m,_?NumberQ,{1}],
    {1},FreeQ[m[[2,3]],_?Negative]/.{True->1,False->-1},
    {2},FreeQ[m[[3,1]],_?Negative]/.{True->1,False->-1},
    {3},FreeQ[m[[1,2]],_?Negative]/.{True->1,False->-1}
  ];
  Sqrt[norm]*Prepend[sign*qV,q0]//Apply[quat]
]


qFromSymbolicIdentityM[m_]:=With[
  {e=First@Diagonal[m]},
  {PowerExpand@Sqrt@e,0,0,0}//Apply[quat]
]


qFrom90\[Degree]AroundBaseAxisSymbolicM[m_]:=Module[
  {e,i,sign},
  e=Last@Sort@Diagonal[m];
  i=First@Flatten@Position[Diagonal@m,e];
  sign=Switch[i,
    1,Sign@(m/e)[[2,3]],
    2,Sign@(m/e)[[3,1]],
    3,Sign@(m/e)[[1,2]]
  ];
  ReplacePart[{PowerExpand@Sqrt[e/2],0,0,0},i+1->sign*PowerExpand@Sqrt[e/2]]//Apply[quat]
]


qFrom120\[Degree]AroundOctantDiagonalSymbolicM[m_]:=Module[
  {e,signs},
  e=Last@Sort@Flatten@m;
  signs={
    1,
    (m/e)[[2,3]]-(m/e)[[3,2]],
    (m/e)[[3,1]]-(m/e)[[1,3]],
    (m/e)[[1,2]]-(m/e)[[2,1]]
  };
  ConstantArray[PowerExpand@Sqrt@e/2,4]*signs//Apply[quat]
]


qFrom180\[Degree]AroundBaseAxisSymbolicM[m_]:=Module[
  {e,i},
  e=Last@Sort@Diagonal[m];
  i=First@Flatten@Position[Diagonal@m,e,{1}];
  ReplacePart[{0,0,0,0},i+1->PowerExpand@Sqrt@e]//Apply[quat]
]


qFrom180\[Degree]AroundPlaneDiagonalSymbolicM[m_]:=Module[
  {e,i,sign,q},
  e=-Last@Sort@Diagonal[m];
  i=First@Flatten@Position[Diagonal@m,-e];
  sign=Sign@First@Total@Cases[ReplacePart[m,{j_,j_}->0]//Flatten,Except[0|0.]];
  q=PowerExpand@ReplacePart[{0,Sqrt[e/2],Sqrt[e/2],Sqrt[e/2]},i+1->0];
  Replace[q,{head:Repeated[0],e1:Except[0],tail__}:>{head,sign*e1,tail}]//Apply[quat]
]


qFrom180\[Degree]AroundAxisInBasePlaneSymbolicM[m_]:=Module[
  {mx=Expand@m,axis,sign},
  axis=PowerExpand[Sqrt[Diagonal@mx]/._?Negative*_Symbol^2->0];
  sign=FirstCase[ReplacePart[mx,{i_,i_}->Nothing],_?NumberQ,"",{3}]//Sign;
  Prepend[MapAt[sign*#&,axis,FirstPosition[axis,Except@0,"",{1},Heads->False]],0]//Apply[quat]
]


qFromEuler2SymbolicAnglesM[m_]:=Module[
  {ax1,ax2,ax3,sign1,sign2,angle1,angle2},
  ax2=First@FirstPosition[m,0];
  ax3=First@First@Position[Diagonal@m,Cos[s_Symbol]*Cos[t_Symbol],{1}];
  ax1=Complement[{1,2,3},{ax2,ax3}]//First;
  sign1=If[MatchQ[m[[ax2,ax3]],-_],-1,1]*Signature[{ax1,ax2,ax3}];
  sign2=If[MatchQ[m[[ax3,ax1]],-_],-1,1]*Signature[{ax1,ax2,ax3}];
  angle1=FirstCase[m[[ax2,ax2]],_Symbol];
  angle2=FirstCase[m[[ax1,ax1]],_Symbol];
  quatToFrom\[Theta]V[angle1,sign1*UnitVector[3,ax1]]**quatToFrom\[Theta]V[angle2,sign2*UnitVector[3,ax2]]
]


qFromSymbolsOnlyM[m_]:=Module[
  {q0,q1,q2,q3},
  q0=Cases[Tr@m,Except[-_]]//Cases[#,_Symbol,-1]&//First;
  q1=DeleteCases[m[[2,3]]-m[[3,2]]//Expand,q0]/4;
  q2=DeleteCases[m[[3,1]]-m[[1,3]]//Expand,q0]/4;
  q3=DeleteCases[m[[1,2]]-m[[2,1]]//Expand,q0]/4;
  {q0,q1,q2,q3}//Apply[quat]
]


qFrom180\[Degree]AroundAxisOutOfBasePlaneSymbolicM[m_]:=Module[
  {mx=Expand@m,axis,signs},
  axis=PowerExpand[Sqrt[Diagonal@mx]/._?Negative*_Symbol^2->0];
  signs=Sign[Times@@@Map[Cases[#,_?NumberQ,{2}]&,ReplacePart[mx,{i_,i_}->Nothing]]];
  Prepend[signs*axis,0]//Apply[quat]
]


qFromSymbolicMRotationInBasePlane[m_]:=Module[
  {mx=Factor@m,norm,q0,syms,signs,qV},
  norm=FirstCase[Diagonal@mx,_?NumberQ,1,{2}];
  q0=FirstCase[Diagonal@mx,Cos[_],"",Infinity];
  syms=Replace[List@@@Diagonal[mx/norm],{{_,-_,-_}->0,{_,x_,-_}:>x,{_,-_,x_}:>x},{1}]/.Sin[_]->1//Sqrt//PowerExpand;
  signs=Switch[FirstPosition[syms,0],
    {1},{0,Sign@First@mx[[3,1]],Sign@First@mx[[1,2]]},
    {2},{Sign@First@mx[[2,3]],0,Sign@First@mx[[1,2]]},
    {3},{Sign@First@mx[[2,3]],Sign@First@mx[[3,1]],0}
  ];
  qV=signs*syms*(q0/.Cos->Sin);
  Sqrt[norm]*Prepend[qV,q0]//Apply[quat]
]


qFromSymbolicMRotationOutOfBasePlanes[m_]:=Module[
  {mx=Factor@m,norm,q0,syms,signs,qV},
  norm=FirstCase[Diagonal@mx,_?NumberQ,1,{2}]; 
  q0=FirstCase[Diagonal@m,Cos[_],"",Infinity];
  syms=Cases[Diagonal[mx/norm]/.{Cos[_]->0,Sin[_]->1,s_Symbol^2:>s},_Symbol,{2}];
  signs=Map[Sign@First@Extract[mx,#]&,{{2,3},{3,1},{1,2}}];
  qV=signs*syms*(q0/.Cos->Sin);
  Sqrt[norm]*Prepend[qV,q0]//Apply[quat]
]


qFromEuler3SymbolicAnglesDistinctAxesM[m_]:=Module[
  {mp=DeleteCases[m,-1,Infinity],ax1,ax2,ax3,sign1,sign2,sign3,angle1,angle2,angle3},
  ax1=Last@First@Position[mp,Sin[t_Symbol],2];
  ax2=First@FirstPosition[mp,Cos[s_Symbol]*Cos[u_Symbol]+Sin[s_Symbol]*Sin[t_Symbol]*Sin[u_Symbol]];
  ax3=First@First@Position[mp,Sin[t_Symbol],2];
  sign1=If[MatchQ[m[[ax3,ax2]],-_],-1,1]*-Signature[{ax1,ax2,ax3}];
  sign2=If[MatchQ[m[[ax3,ax1]],-_],-1,1]*Signature[{ax1,ax2,ax3}];
  sign3=If[MatchQ[m[[ax2,ax1]],-_],-1,1]*-Signature[{ax1,ax2,ax3}];
  angle2=Cases[mp,Sin[x_]:>x,{2}]//First;
  angle1=FirstCase[DeleteDuplicates[Cases[m[[ax3]],_Symbol,{-1}]],Except@angle2];
  angle3=Complement[DeleteDuplicates[Cases[m,_Symbol,{-1}]],{angle1,angle2}]//First;
  quatToFrom\[Theta]V[angle1,sign1*UnitVector[3,ax1]]**quatToFrom\[Theta]V[angle2,sign2*UnitVector[3,ax2]]**quatToFrom\[Theta]V[angle3,sign3*UnitVector[3,ax3]]
]


qFromEuler3SymbolicAnglesRepeatedAxisM[m_]:=Module[
  {mp=DeleteCases[m,-1,Infinity],ax1,ax2,ax3,sign1,sign2,sign3,angle1,angle2,angle3},
  ax1=First@First@Position[Diagonal@m,Cos[_Symbol],{1}];
  ax2=First@First@Position[Diagonal@mp,Cos[s_Symbol]*Cos[u_Symbol]+Cos[t_Symbol]*Sin[s_Symbol]*Sin[u_Symbol],{1}];
  ax3=First@First@Position[Diagonal@mp,Cos[s_Symbol]*Cos[t_Symbol]*Cos[u_Symbol]+Sin[s_Symbol]*Sin[u_Symbol],{1}];
  sign1=If[Count[m[[ax1]],-1,Infinity]==1,1,-1]*Signature[{ax1,ax2,ax3}];
  sign2=If[MatchQ[m[[ax1,ax3]],-_],1,-1]*Signature[{ax1,ax2,ax3}];
  sign3=If[Count[m[[All,ax1]],-1,Infinity]==1,-1,1]*Signature[{ax1,ax2,ax3}];
  angle2=Cases[m,Cos[x_]:>x,{2}]//First;
  angle1=FirstCase[Cases[m[[ax1]],_Symbol,{-1}],Except[angle2]];
  angle3=FirstCase[Cases[m[[All,ax1]],_Symbol,{-1}],Except[angle2]];
  quatToFrom\[Theta]V[angle1,sign1*UnitVector[3,ax1]]**quatToFrom\[Theta]V[angle2,sign2*UnitVector[3,ax2]]**quatToFrom\[Theta]V[angle3,sign3*UnitVector[3,ax1]]
]


signedAngleBetweenVectors[startV_,endV_,axis_]:=Module[
  {sinvalue,cosvalue,angle},
  sinvalue=Cross[Normalize@startV,Normalize@endV]//Norm;
  cosvalue=Dot[Normalize@startV,Normalize@endV];
  angle=Which[
    sinvalue<=cosvalue,ArcSin@sinvalue,
    Abs@cosvalue<sinvalue,ArcCos@cosvalue,
    True,\[Pi]-ArcSin@sinvalue
  ];
  Replace[Sign@Dot[Cross[startV,endV],axis],0->1]*angle
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
  angles[[1]]={z,y,x}/\[Degree]//N//Simplify//roundNumbers;
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
        x_*Sin[a_]/Sqrt[1+3 Cos[\[Theta]_]^2-x_^2*Sin[\[Theta]_]^2-y_^2*Sin[\[Theta]_]^2]:>x*Sin[\[Theta]],sqAbsRule
      },
    _,q
  ]
]


mOut[m_]:=If[!MemberQ[{46,47},mType[m]],Simplify@m,m]//roundNumbers[#]&


\[Theta]VOut[rotIn_List]:=With[
  {rot=roundNumbers[rotIn]},
  Simplify[PowerExpand[rot]/. 1/Sqrt[1-Cos[\[Theta]_]^2]:>1/Sin[\[Theta]]]/.sqAbsRule
]


roundNumbers[x_]:=ReplaceAll[x,n_?MachineNumberQ:>If[Abs[n-Round@n]<10^-12,Round@n,n]]


sqAbsRule:=Abs[s_]^p:2|-2:>s^p


(* ::Subsection::Closed:: *)
(*Version date*)


quatVersionDate="2026-01-12";


(* ::Section::Closed:: *)
(*End*)


End[]


SyntaxInformation[quat]={"ArgumentsPattern"->{_,_,_,_}};


EndPackage[]
