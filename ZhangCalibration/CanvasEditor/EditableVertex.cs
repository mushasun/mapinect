using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Shapes;
using PropertyChangedLibrary;
using System;

namespace CanvasEditor
{
	internal class EditableVertex : EditableAlteringObject
	{

		internal EditableVertex(EditableAlteringManager manager, int vertexIndex)
			: base(manager)
		{
			VertexIndex = vertexIndex;
		}

		public int VertexIndex { get; private set; }

		internal override void UpdatePosition()
		{
			
		}

		protected override FrameworkElement NewAlteringFrameworkElement()
		{
			return new Ellipse();
		}

	}

}
