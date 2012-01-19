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

		private EditableHandleManager HandleManager { get; set; }

		public bool Select {
			set
			{
				HandleManager.HandlesVisible = value;
			}
		}

		public PointNotifyPropertyChanged Position { get; private set; }

		public SizeNotifyPropertyChanged Size { get; private set; }

		public event OnEditablePositionChanged PositionChanged;

		public event OnEditableSizeChanged SizeChanged;


	}
}
