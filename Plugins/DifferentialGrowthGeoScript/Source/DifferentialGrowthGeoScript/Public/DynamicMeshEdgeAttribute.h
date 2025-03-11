


#pragma once

#include "UDynamicMesh.h"

namespace UE
{
namespace Geometry
{

template<typename AttribValueType, int AttribDimension>
class TDynamicMeshEdgeAttribute;


template<typename AttribValueType, int AttribDimension>
class FDynamicMeshEdgeAttributeChange : public FDynamicMeshAttributeChangeBase
{
private:
	struct FChangeEdgeAttribute
	{
		AttribValueType Data[AttribDimension];
		int EdgeID;
	};

	TArray<FChangeEdgeAttribute> OldEdgeAttributes, NewEdgeAttributes;

public:
	FDynamicMeshEdgeAttributeChange()
	{}

	virtual ~FDynamicMeshEdgeAttributeChange()
	{}

	inline virtual void SaveInitialEdge(const FDynamicMeshAttributeBase* Attribute, int EdgeID) override;

	inline virtual void StoreAllFinalEdges(const FDynamicMeshAttributeBase* Attribute, const TArray<int>& EdgeIDs) override;

	inline virtual bool Apply(FDynamicMeshAttributeBase* Attribute, bool bRevert) const override;
};



/**
 * TDynamicMeshEdgeAttribute is an add-on to a FDynamicMesh3 that allows for
 * per-edge storage of an attribute value.
 *
 * The FDynamicMesh3 mesh topology operations (eg split/flip/collapse edge, poke face, etc)
 * can be mirrored to the overlay via OnSplitEdge(), etc.
 */
template<typename AttribValueType, int AttribDimension>
class TDynamicMeshEdgeAttribute : public FDynamicMeshAttributeBase
{

protected:
	/** The parent mesh this overlay belongs to */
	FDynamicMesh3* ParentMesh;

	/** List of per-Edge attribute values */
	TDynamicVector<AttribValueType> AttribValues;

	using Super = FDynamicMeshAttributeBase;

	friend class FDynamicMesh3;
	friend class FDynamicMeshAttributeSet;

public:
	/** Create an empty overlay */
	TDynamicMeshEdgeAttribute()
	{
		ParentMesh = nullptr;
	}

	/** Create an overlay for the given parent mesh */
	TDynamicMeshEdgeAttribute(FDynamicMesh3* ParentMeshIn, bool bAutoInit = true)
	{
		ParentMesh = ParentMeshIn;
		if (bAutoInit)
		{
			Initialize();
		}
	}

private:
	/** @set the parent mesh for this overlay.  Only safe for use during FDynamicMesh move */
	void Reparent(FDynamicMesh3* ParentMeshIn)
	{
		ParentMesh = ParentMeshIn;
	}

public:
	/** @return the parent mesh for this overlay */
	const FDynamicMesh3* GetParentMesh() const { return ParentMesh; }
	/** @return the parent mesh for this overlay */
	FDynamicMesh3* GetParentMesh() { return ParentMesh; }

	virtual FDynamicMeshAttributeBase* MakeNew(FDynamicMesh3* ParentMeshIn) const override
	{
		TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>* Matching = new TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>(ParentMeshIn);
		Matching->Initialize();
		return Matching;
	}

	virtual FDynamicMeshAttributeBase* MakeCopy(FDynamicMesh3* ParentMeshIn) const override
	{
		TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>* ToFill = new TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>(ParentMeshIn);
		ToFill->Copy(*this);
		return ToFill;
	}

	/** Set this overlay to contain the same arrays as the copy overlay */
	void Copy(const TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>& Copy)
	{
		CopyParentClassData(Copy);
		AttribValues = Copy.AttribValues;
	}

	virtual FDynamicMeshAttributeBase* MakeCompactCopy(const FCompactMaps& CompactMaps, FDynamicMesh3* ParentMeshIn) const override
	{
		TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>* ToFill = new TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>(ParentMeshIn);
		ToFill->Initialize();
		ToFill->CompactCopy(CompactMaps, *this);
		return ToFill;
	}


	/** Initialize the attribute values with InitialValue, and resize to the parent mesh's max edge ID */
	void Initialize(AttribValueType InitialValue = (AttribValueType)0)
	{
		check(ParentMesh != nullptr);
		AttribValues.Resize(ParentMesh->MaxEdgeID() * AttribDimension);
		AttribValues.Fill(InitialValue);
	}

	void SetNewValue(int NewEdgeID, const AttribValueType* Data)
	{
		int k = NewEdgeID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			AttribValues.InsertAt(Data[i], k + i);
		}
	}



	//
	// Accessors/Queries
	//  


	virtual bool CopyOut(int RawID, void* Buffer, int BufferSize) const override
	{
		if (sizeof(AttribValueType) * AttribDimension != BufferSize)
		{
			return false;
		}
		AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
		int k = RawID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			BufferData[i] = AttribValues[k + i];
		}
		return true;
	}
	virtual bool CopyIn(int RawID, void* Buffer, int BufferSize) override
	{
		if (sizeof(AttribValueType) * AttribDimension != BufferSize)
		{
			return false;
		}
		AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
		int k = RawID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			AttribValues[k + i] = BufferData[i];
		}
		return true;
	}

	/** Get the element at a given index */
	inline void GetValue(int EdgeID, AttribValueType* Data) const
	{
		int k = EdgeID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			Data[i] = AttribValues[k + i];
		}
	}

	/** Get the element at a given index */
	template<typename AsType>
	void GetValue(int EdgeID, AsType& Data) const
	{
		int k = EdgeID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			Data[i] = AttribValues[k + i];
		}
	}


	/** Set the element at a given index */
	inline void SetValue(int EdgeID, const AttribValueType* Data)
	{
		int k = EdgeID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			AttribValues[k + i] = Data[i];
		}
	}

	/** Set the element at a given index */
	template<typename AsType>
	void SetValue(int EdgeID, const AsType& Data)
	{
		int k = EdgeID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			AttribValues[k + i] = Data[i];
		}
	}

	/**
	 * Copy the attribute value at FromEdgeID to ToEdgeID
	 */
	inline void CopyValue(int FromEdgeID, int ToEdgeID)
	{
		int kA = FromEdgeID * AttribDimension;
		int kB = ToEdgeID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			AttribValues.InsertAt(AttribValues[kA + i], kB + i);
		}
	}


	virtual TUniquePtr<FDynamicMeshAttributeChangeBase> NewBlankChange() const override
	{
		return MakeUnique<FDynamicMeshEdgeAttributeChange<AttribValueType, AttribDimension>>();
	}


public:

	/** Update the overlay to reflect an edge split in the parent mesh */
	void OnSplitEdge(const DynamicMeshInfo::FEdgeSplitInfo& SplitInfo) override
	{
		OnNewEdge(SplitInfo.NewEdges.A);
		OnNewEdge(SplitInfo.NewEdges.B);
		CopyValue(SplitInfo.OriginalEdge, SplitInfo.NewEdges.A);
		CopyValue(SplitInfo.OriginalEdge, SplitInfo.NewEdges.B);
		if (SplitInfo.NewEdges.C != FDynamicMesh3::InvalidID)
		{
			OnNewEdge(SplitInfo.NewEdges.C);
			CopyValue(SplitInfo.OriginalEdge, SplitInfo.NewEdges.C);
		}
	}

	/** Update the overlay to reflect an edge flip in the parent mesh */
	void OnFlipEdge(const DynamicMeshInfo::FEdgeFlipInfo& FlipInfo) override
	{
		// edge just moved so we will leave attrib unmodified
	}

	/** Update the overlay to reflect an edge collapse in the parent mesh */
	void OnCollapseEdge(const DynamicMeshInfo::FEdgeCollapseInfo& CollapseInfo) override
	{
		// nothing to do here, edges were only deleted
	}

	/** Update the overlay to reflect a face poke in the parent mesh */
	void OnPokeTriangle(const DynamicMeshInfo::FPokeTriangleInfo& PokeInfo) override
	{
		// TODO
		//CopyValue(PokeInfo.OriginalTriangle, PokeInfo.NewTriangles.A);
		//CopyValue(PokeInfo.OriginalTriangle, PokeInfo.NewTriangles.B);
	}

	/** Update the overlay to reflect an edge merge in the parent mesh */
	void OnMergeEdges(const DynamicMeshInfo::FMergeEdgesInfo& MergeInfo) override
	{
		// TODO blend values
		// nothing to do here because triangles did not change
	}

	void OnMergeVertices(const DynamicMeshInfo::FMergeVerticesInfo& MergeInfo) override
	{
		// TODO
		// This resolves as either an edge collapse, edge weld, or merge of disconnected vertices. 
		//  The triangles either get removed or unchanged- nothing more to do here.
	}

	/** Update the overlay to reflect a vertex split in the parent */
	void OnSplitVertex(const DynamicMeshInfo::FVertexSplitInfo& SplitInfo, const TArrayView<const int>& TrianglesToUpdate) override
	{
		// TODO
		// nothing to do here because triangles did not change
	}

	virtual AttribValueType GetDefaultAttributeValue()
	{
		return (AttribValueType)0;
	}

	inline void ResizeAttribStoreIfNeeded(int EdgeID)
	{
		if (!ensure(EdgeID >= 0))
		{
			return;
		}
		size_t NeededSize = (((size_t)EdgeID + 1) * AttribDimension);
		if (NeededSize > AttribValues.Num())
		{
			AttribValues.Resize(NeededSize, GetDefaultAttributeValue());
		}
	}

	virtual void OnNewTriangle(int TriangleID, bool bInserted) override
	{
		/// TODO
		//check(ParentMesh != nullptr);
		//FIndex3i Triangle = ParentMesh->GetTriangle(TriangleID);
		//int32 MaxEdgeID = FMath::Max3(Triangle.A, Triangle.B, Triangle.C);
		//ResizeAttribStoreIfNeeded(MaxEdgeID);
	}

	void OnNewEdge(int EdgeID)
	{
		ResizeAttribStoreIfNeeded(OnNewEdge);
	}

public:
	/**
	 * Returns true if this AttributeSet is the same as Other.
	 */
	bool IsSameAs(const TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>& Other, bool bIgnoreDataLayout) const
	{
		if (!bIgnoreDataLayout)
		{
			if (AttribValues.Num() != Other.AttribValues.Num())
			{
				return false;
			}

			for (int Idx = 0, NumValues = AttribValues.Num(); Idx < NumValues; Idx++)
			{
				if (AttribValues[Idx] != Other.AttribValues[Idx])
				{
					return false;
				}
			}
		}
		else
		{
			FRefCountVector::IndexIterator ItTid = ParentMesh->GetEdgesRefCounts().BeginIndices();
			const FRefCountVector::IndexIterator ItTidEnd = ParentMesh->GetEdgesRefCounts().EndIndices();
			FRefCountVector::IndexIterator ItTidOther = Other.ParentMesh->GetEdgesRefCounts().BeginIndices();
			const FRefCountVector::IndexIterator ItTidEndOther = Other.ParentMesh->GetEdgesRefCounts().EndIndices();

			while (ItTid != ItTidEnd && ItTidOther != ItTidEndOther)
			{
				for (int32 i = 0; i < AttribDimension; ++i)
				{
					const AttribValueType AttribValue = AttribValues[*ItTid * AttribDimension + i];
					const AttribValueType AttribValueOther = Other.AttribValues[*ItTidOther * AttribDimension + i];
					if (AttribValue != AttribValueOther)
					{
						// Edge attribute value is not the same.
						return false;
					}
				}
				++ItTid;
				++ItTidOther;
			}

			if (ItTid != ItTidEnd || ItTidOther != ItTidEndOther)
			{
				// Number of edge attribute values is not the same.
				return false;
			}
		}

		return true;
	}

	/**
	 * Serialization operator for TDynamicMeshEdgeAttribute.
	 *
	 * @param Ar Archive to serialize with.
	 * @param Attr Mesh edge attribute to serialize.
	 * @returns Passing down serializing archive.
	 */
	friend FArchive& operator<<(FArchive& Ar, TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>& Attr)
	{
		Attr.Serialize(Ar, nullptr, false);
		return Ar;
	}

	/**
	* Serialize to and from an archive.
	*
	* @param Ar Archive to serialize with.
	* @param CompactMaps If this is not a null pointer, the mesh serialization compacted the vertex and/or triangle data using the provided mapping.
	* @param bUseCompression Use compression for serializing bulk data.
	*/
	void Serialize(FArchive& Ar, const FCompactMaps* CompactMaps, bool bUseCompression)
	{
		Super::Serialize(Ar);

		Ar.UsingCustomVersion(FUE5MainStreamObjectVersion::GUID);
		if (Ar.IsLoading() && Ar.CustomVer(FUE5MainStreamObjectVersion::GUID) < FUE5MainStreamObjectVersion::DynamicMeshCompactedSerialization)
		{
			Ar << AttribValues;
		}
		else
		{
			auto SerializeVector = [](FArchive& Ar, auto& Vector, bool bUseCompression)
				{
					if (bUseCompression)
					{
						Vector.template Serialize<true, true>(Ar);
					}
					else
					{
						Vector.template Serialize<true, false>(Ar);
					}
				};

			Ar << bUseCompression;

			SerializeVector(Ar, AttribValues, bUseCompression);
		}
	}
};




template<typename AttribValueType, int AttribDimension>
void FDynamicMeshEdgeAttributeChange<AttribValueType, AttribDimension>::SaveInitialEdge(const FDynamicMeshAttributeBase* Attribute, int EdgeID)
{
	FChangeEdgeAttribute& Change = OldEdgeAttributes.Emplace_GetRef();
	Change.EdgeID = EdgeID;
	const TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>* AttribCast = static_cast<const TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>*>(Attribute);
	AttribCast->GetValue(EdgeID, Change.Data);
}

template<typename AttribValueType, int AttribDimension>
void FDynamicMeshEdgeAttributeChange<AttribValueType, AttribDimension>::StoreAllFinalEdges(const FDynamicMeshAttributeBase* Attribute, const TArray<int>& EdgeIDs)
{
	const TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>* AttribCast = static_cast<const TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>*>(Attribute);
	NewEdgeAttributes.Reserve(NewEdgeAttributes.Num() + EdgeIDs.Num());
	for (int EdgeID : EdgeIDs)
	{
		FChangeEdgeAttribute& Change = NewEdgeAttributes.Emplace_GetRef();
		Change.EdgeID = EdgeID;
		AttribCast->GetValue(EdgeID, Change.Data);
	}
}

template<typename AttribValueType, int AttribDimension>
bool FDynamicMeshEdgeAttributeChange<AttribValueType, AttribDimension>::Apply(FDynamicMeshAttributeBase* Attribute, bool bRevert) const
{
	TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>* AttribCast = static_cast<TDynamicMeshEdgeAttribute<AttribValueType, AttribDimension>*>(Attribute);
	const TArray<FChangeEdgeAttribute>* Changes = bRevert ? &OldEdgeAttributes : &NewEdgeAttributes;
	for (const FChangeEdgeAttribute& Change : *Changes)
	{
		bool bIsEdge = AttribCast->GetParentMesh()->IsEdge(Change.EdgeID);
		checkSlow(bIsEdge);
		if (bIsEdge)
		{
			AttribCast->SetValue(Change.EdgeID, Change.Data);
		}
	}
	return true;
}


} // end namespace UE::Geometry
} // end namespace UE
