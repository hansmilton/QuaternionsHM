(* ::Package:: *)

NotebookSave[
  CreatePalette[
    Grid[{
      {Spacer@2,SpanFromLeft,SpanFromLeft},
      {
        PasteButton[Tooltip["quat","Quaternion template"],quat[\[Placeholder],\[Placeholder],\[Placeholder],\[Placeholder]]],
        PasteButton[Tooltip["Q\[Cross]Q","Multiplication of quaternions"],\[SelectionPlaceholder]**\[Placeholder]],
        PasteButton[Tooltip["\!\(\*SuperscriptBox[\(Q\), \(-1\)]\)","Reciprocal of quaternion"],(Power[\[SelectionPlaceholder],-1])]
      },
      {
        PasteButton[Tooltip["Q\[LeftRightArrow]{}","Quaternion \[LeftRightArrow] List"],quatToFromList[\[SelectionPlaceholder]]],
        PasteButton[Tooltip["Q\[LeftRightArrow]\[Theta]V","Quaternion \[LeftRightArrow] Angle/Axis"],quatToFrom\[Theta]V[\[SelectionPlaceholder]]],
        PasteButton[Tooltip["Q\[LeftRightArrow]M","Quaternion \[LeftRightArrow] Matrix"],quatToFromMatrix[\[SelectionPlaceholder]]]
      },
      {
        PasteButton[Tooltip["Q\[Cross]V","Quaternion rotation of a vector"],quatRotateVector[\[SelectionPlaceholder],\[Placeholder]]],
        PasteButton[Tooltip["Q\[LeftRightArrow]EUL","Quaternion \[LeftRightArrow] Euler ZYX angles"],quatToFromEulerZYX[\[SelectionPlaceholder]]],
        PasteButton[Tooltip["AlignedM\[RightArrow]Q","Quaternion from interactive spec of matrix"],quatFromAlignedMatrix[]]
      },
      {
        Button[
          "Inbuilt functions info",
          CreatePalette[
            Panel[
              Column[{
                "Inbuilt functions that can take \!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\) as argument:",
                "- Non commutative multiplication, \!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)**\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\).",
                "- Power, \!\(\*SuperscriptBox[
StyleBox[\"quat\",\nFontSlant->\"Italic\"], 
StyleBox[\"p\",\nFontSlant->\"Italic\"]]\). The exponent \!\(\*
StyleBox[\"p\",\nFontSlant->\"Italic\"]\) has to be a scalar.",
                "- Conjugate",
                "- Norm",
                "- Normalize",
                "- Exp",
                "- Log",
                "- Times, scalar*\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)",
                "- Plus, scalar+\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\) and \!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)+\!\(\*
StyleBox[\"quat\",\nFontSlant->\"Italic\"]\)"
              }],
              Background->LightBlue,BaseStyle->14
            ],
            WindowTitle->None,WindowMargins->Automatic,Saveable->False
          ]
        ],
        SpanFromLeft,SpanFromLeft
      },{"","",""}
    }],
    WindowTitle->"Quaternions Palette",WindowElements->{"StatusArea","MagnificationPopUp"}
  ],
  Interactive->True
]
