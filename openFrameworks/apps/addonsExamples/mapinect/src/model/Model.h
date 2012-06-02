#ifndef MAPINECT_Model_H__
#define MAPINECT_Model_H__

#include <vector>

#include "IObject.h"
#include "ModelObject.h"
#include "ofxMutex.h"
#include "Table.h"

namespace mapinect
{
	class Model
	{
	public:
		Model();

		// non-thread-safe
		inline const vector<ModelObjectPtr>&	getObjects() const	{ return objects; }
		inline const TablePtr&					getTable() const	{ return table; }

		// thread-safe
		PCPtr									getCloudSum() const;
		vector<IObjectPtr>						getMathModelApproximation() const;

		void addObject(const ModelObjectPtr&);
		void removeObject(const ModelObjectPtr&);
		void resetObjects();
		void setTable(const TablePtr&);
		void resetTable();

		ofxMutex						objectsMutex;
		ofxMutex						tableMutex;

	private:
		vector<ModelObjectPtr>			objects;
		TablePtr						table;
	};
}

#endif	// PCM_H__
