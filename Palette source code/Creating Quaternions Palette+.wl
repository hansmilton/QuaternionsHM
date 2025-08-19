(* ::Package:: *)

NotebookSave[
  CreatePalette[
    DynamicModule[
      {loaded},
      Grid[{
        {
          Dynamic[
            If[loaded,
              Spacer@5,
              Button[
                "Load package",
                Needs@"QuaternionsHM`";loaded=True,Background->White
              ]
            ],
            Initialization:>(loaded=Or[MemberQ[$ContextPath,"QuaternionsHM`"],Options[InputNotebook[],CellContext]==={CellContext->Notebook}])
          ],
          SpanFromLeft,SpanFromLeft
        },
        {
          PasteButton[Tooltip["quat","Quaternion template"],quat[\[Placeholder],\[Placeholder],\[Placeholder],\[Placeholder]],Enabled->Dynamic@loaded],
          PasteButton[Tooltip["Q\[Cross]Q","Multiplication of quaternions"],\[SelectionPlaceholder]**\[Placeholder],Enabled->Dynamic@loaded],
          PasteButton[Tooltip["\!\(\*SuperscriptBox[\(Q\), \(-1\)]\)","Reciprocal of quaternion"],(Power[\[SelectionPlaceholder],-1]),Enabled->Dynamic@loaded]
        },
        {
          PasteButton[Tooltip["Q\[LeftRightArrow]{}","Quaternion \[LeftRightArrow] List"],quatToFromList[\[SelectionPlaceholder]],Enabled->Dynamic@loaded],
          PasteButton[Tooltip["Q\[LeftRightArrow]\[Theta]V","Quaternion \[LeftRightArrow] Angle/Axis"],quatToFrom\[Theta]V[\[SelectionPlaceholder]],Enabled->Dynamic@loaded],
          PasteButton[Tooltip["Q\[LeftRightArrow]M","Quaternion \[LeftRightArrow] Matrix"],quatToFromMatrix[\[SelectionPlaceholder]],Enabled->Dynamic@loaded]
        },
        {
          PasteButton[Tooltip["Q\[Cross]V","Quaternion rotation of a vector"],quatRotateVector[\[SelectionPlaceholder],\[Placeholder]],Enabled->Dynamic@loaded],
          PasteButton[Tooltip["Q\[LeftRightArrow]EUL","Quaternion \[LeftRightArrow] Euler ZYX angles"],quatToFromEulerZYX[\[SelectionPlaceholder]],Enabled->Dynamic@loaded],
          PasteButton[Tooltip["AlignedM\[RightArrow]Q","Quaternion from interactive spec of matrix"],quatFromAlignedMatrix[],Enabled->Dynamic@loaded]
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
      },Alignment->Center],
      Initialization:>(loaded=False)
    ],
    WindowTitle->"Quaternions Palette+",WindowElements->{"StatusArea","MagnificationPopUp"}
  ],
  Interactive->True
]
