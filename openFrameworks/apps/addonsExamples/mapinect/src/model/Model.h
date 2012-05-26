#ifndef MAPINECT_Model_H__
#define MAPINECT_Model_H__

#include "ModelObject.h"
#include "ofxMutex.h"
#include <vector>
#include "Table.h"

namespace mapinect {
	class Model {

	public:
		Model();

		inline const vector<ModelObjectPtr>&	getObjects()	{ return objects; }
		inline const TablePtr&					getTable()		{ return table; }

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
