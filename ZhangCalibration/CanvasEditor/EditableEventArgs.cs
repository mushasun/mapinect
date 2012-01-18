using System;
using System.Windows;

namespace CanvasEditor
{
	public class EditableEventArgs : EventArgs
	{

		public EditableEventArgs(IEditable editable)
		{
			Editable = editable;
		}

		public IEditable Editable { get; private set; }

	}
}
