using System;
using System.Windows;
using PropertyChangedLibrary;

namespace CanvasEditor
{
	public interface IEditable
	{
		UIElement UIElement { get; }
		bool IsEnabled { get; }
		bool Select { set; }

		PointNotifyPropertyChanged Position { get; }
		SizeNotifyPropertyChanged Size { get; }

		event OnEditablePositionChanged PositionChanged;
		event OnEditableSizeChanged SizeChanged;
	}

	public delegate void OnEditablePositionChanged(IEditable sender, EventArgs e);
	public delegate void OnEditableSizeChanged(IEditable sender, EventArgs e);
}
