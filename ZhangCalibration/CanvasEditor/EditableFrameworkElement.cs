using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using PropertyChangedLibrary;

namespace CanvasEditor
{
	class EditableFrameworkElement : IEditable
	{

		EditableFrameworkElement(CanvasEditor editor, FrameworkElement element)
		{
			Editor = editor;
			UIElement = element;
		}

		public CanvasEditor Editor { get; private set; }

		public UIElement UIElement { get; private set; }

		private FrameworkElement FrameworkElement
		{
			get
			{
				return (FrameworkElement)UIElement;
			}
		}

		public bool IsEnabled { get; set; }

		public ICollection<IEditableHandle> Handles { get; private set; }

		private void CreateHandlesIfNecessary()
		{
			if (Handles == null)
			{
				Handles = new List<IEditableHandle>();
				for (BoundaryPosition position = 0; position < BoundaryPosition.BoundaryPositionCount; position++)
				{
					IEditableHandle handle = new HandleEditable(this, position);
					Handles.Add(handle);
				}
			}
		}

		private void ShowHandles()
		{
			foreach (IEditableHandle handle in Handles)
			{
				handle.IsVisible = true;
			}
		}

		private bool myIsEditable;
		public bool IsEditable {
			get
			{
				return myIsEditable;
			}
			set
			{
				myIsEditable = value;
				if (myIsEditable)
				{
					CreateHandlesIfNecessary();
					ShowHandles();
				}
			}
		}

		public PointNotifyPropertyChanged Position { get; private set; }

		public SizeNotifyPropertyChanged Size { get; private set; }

		public event OnEditableSizeChanged SizeChanged;


	}
}
