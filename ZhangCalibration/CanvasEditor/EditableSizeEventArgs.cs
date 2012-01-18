using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;

namespace CanvasEditor
{
	public class EditableSizeEventArgs : EditableEventArgs
	{

		public EditableSizeEventArgs(IEditable editable, Size size) : base(editable)
		{
			Size = size;
		}

		public Size Size { get; private set; }

	}
}
