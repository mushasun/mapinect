using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;
using System.Windows;

namespace CanvasEditor
{
	public class CanvasEditor
	{

		public CanvasEditor(Canvas canvas)
		{
			Canvas = canvas;
			Canvas.MouseMove += new System.Windows.Input.MouseEventHandler(canvas_MouseMove);
		}

		public CanvasEditor(System.Windows.Controls.Canvas canvas,
			UIElementCollection editableElements)
			: this(canvas)
		{
			EditableUIElements = editableElements;
		}

		void canvas_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
		{
			System.Windows.Point mousePosition = e.GetPosition(Canvas);
			if (EditingElement != null)
			{
				double left = (double)EditingElement.GetValue(System.Windows.Controls.Canvas.LeftProperty);
				double top = (double)EditingElement.GetValue(System.Windows.Controls.Canvas.TopProperty);
				System.Windows.Vector difference = System.Windows.Point.Subtract(mousePosition, LastMousePosition);
				EditingElement.SetValue(System.Windows.Controls.Canvas.LeftProperty, left + difference.X);
				EditingElement.SetValue(System.Windows.Controls.Canvas.TopProperty, top + difference.Y);
			}
			LastMousePosition = mousePosition;
		}

		void element_PreviewMouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
		{
			if (EditingElement != null && EditableUIElements.Contains((UIElement)sender))
			{
				EditingElement = (System.Windows.FrameworkElement)sender;
			}
		}

		internal void AddEditable(IEditable editable)
		{
			Canvas.Children.Add(editable.UIElement);
		}

		public System.Windows.Controls.Canvas Canvas { get; set; }

		public void Clear()
		{
			Canvas.Children.Clear();
			if (EditableUIElements != null)
			{
				EditableUIElements.Clear();
			}
		}

		public List<IEditable> IEditableElements { get; private set; }

		private UIElementCollection myEditableElements;
		public UIElementCollection EditableUIElements
		{
			get
			{
				return myEditableElements;
			}
			set
			{
				myEditableElements = value;

				IEditableElements = new List<IEditable>();
				foreach (System.Windows.FrameworkElement element in EditableUIElements)
				{
					IEditable editable = EditableWrapper.CreateWrapperForUIElement(this, element);
					if (editable != null)
					{
						IEditableElements.Add(editable);
						element.PreviewMouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(element_PreviewMouseLeftButtonDown);
					}
				}
			}
		}

		public void SelectByUIElement(UIElement element)
		{
			IEditableElements.ForEach(iee => iee.Select = false);
			if (EditableUIElements.Contains(element))
			{
				IEditableElements.First(iee => iee.UIElement == element).Select = true;
			}
		}

		private System.Windows.FrameworkElement EditingElement { get; set; }

		private System.Windows.Point LastMousePosition { get; set; }

	}
}
