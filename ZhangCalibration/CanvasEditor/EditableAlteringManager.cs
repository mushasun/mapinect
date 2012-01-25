using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CanvasEditor
{
	internal abstract class EditableAlteringManager
	{

		internal EditableAlteringManager(IEditable editable)
		{
			Editable = editable;
			AlteringObjectsVisible = false;
		}

		protected IEditable Editable { get; private set; }

		private bool myAlteringObjectsVisible;
		public bool AlteringObjectsVisible
		{
			get
			{
				return myAlteringObjectsVisible;
			}
			set
			{
				myAlteringObjectsVisible = value;
				if (myAlteringObjectsVisible)
				{
					CreateAlteringObjectsIfNecessary();
					UpdateAlteringObjectsPosition();
				}
				if (AlteringObjects != null)
				{
					AlteringObjects.ForEach(h => h.IsVisible = myAlteringObjectsVisible);
				}
			}
		}

		private List<EditableAlteringObject> AlteringObjects { get; set; }

		abstract protected List<EditableAlteringObject> GetAlteringObjects();

		private void CreateAlteringObjectsIfNecessary()
		{
			if (AlteringObjects == null)
			{
				AlteringObjects = GetAlteringObjects();
			}
		}

		private void UpdateAlteringObjectsPosition()
		{
			AlteringObjects.ForEach(h => h.UpdatePosition());
		}

	}
}
