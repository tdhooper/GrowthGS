
namespace UE {
namespace Geometry {

/**
	* TDynamicMeshScalarVertexAttribute is an extension of TDynamicMeshVertexAttribute for scalar-valued attributes.
	* Adds some convenience functions to simplify get/set code.
	*/
template<typename RealType>
class TDynamicMeshScalarVertexAttribute : public TDynamicMeshVertexAttribute<RealType, 1>
{
public:
	using BaseType = TDynamicMeshVertexAttribute<RealType, 1>;
	using BaseType::SetNewValue;
	using BaseType::GetValue;
	using BaseType::SetValue;

	TDynamicMeshScalarVertexAttribute() : BaseType()
	{
	}

	TDynamicMeshScalarVertexAttribute(FDynamicMesh3* ParentMeshIn) : BaseType(ParentMeshIn)
	{
	}

	inline void SetNewValue(int NewVertexID, RealType Value)
	{
		this->AttribValues.InsertAt(Value, NewVertexID);
	}

	inline RealType GetValue(int VertexID) const
	{
		return this->AttribValues[VertexID];
	}

	inline void SetValue(int VertexID, RealType Value)
	{
		this->AttribValues[VertexID] = Value;
	}
};

} // end namespace UE::Geometry
} // end namespace UE