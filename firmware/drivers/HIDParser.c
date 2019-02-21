/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#define  __INCLUDE_FROM_USB_DRIVER
#define  __INCLUDE_FROM_HID_DRIVER

#include <inttypes.h>
#include <string.h>
#include "HIDParser.h"

uint8_t USB_ProcessHIDReport(const uint8_t* ReportData,
                             uint16_t ReportSize,
                             HID_ReportInfo_t* const ParserData)
{
	HID_StateTable_t      StateTable[HID_STATETABLE_STACK_DEPTH];
	HID_StateTable_t*     CurrStateTable     = &StateTable[0];
	HID_CollectionPath_t* CurrCollectionPath = NULL;
	HID_ReportSizeInfo_t* CurrReportIDInfo   = &ParserData->ReportIDSizes[0];
	uint16_t              UsageList[HID_USAGE_STACK_DEPTH];
	uint8_t               UsageListSize      = 0;
	HID_MinMax_t          UsageMinMax        = {0, 0};
    uint8_t i;
    uint8_t ReportItemNum;

	memset(ParserData,       0x00, sizeof(HID_ReportInfo_t));
	memset(CurrStateTable,   0x00, sizeof(HID_StateTable_t));
	memset(CurrReportIDInfo, 0x00, sizeof(HID_ReportSizeInfo_t));

	ParserData->TotalDeviceReports = 1;

	while (ReportSize)
	{
		uint8_t  HIDReportItem  = *ReportData;
		uint32_t ReportItemData;

		ReportData++;
		ReportSize--;
        
		switch (HIDReportItem & HID_RI_DATA_SIZE_MASK)
		{
			case HID_RI_DATA_BITS_32:
				ReportItemData  = (((uint32_t)ReportData[3] << 24) | ((uint32_t)ReportData[2] << 16) |
			                       ((uint16_t)ReportData[1] << 8)  | ReportData[0]);
				ReportSize     -= 4;
				ReportData     += 4;
				break;

			case HID_RI_DATA_BITS_16:
				ReportItemData  = (((uint16_t)ReportData[1] << 8) | (ReportData[0]));
				ReportSize     -= 2;
				ReportData     += 2;
				break;

			case HID_RI_DATA_BITS_8:
				ReportItemData  = ReportData[0];
				ReportSize     -= 1;
				ReportData     += 1;
				break;

			default:
				ReportItemData  = 0;
				break;
		}

		switch (HIDReportItem & (HID_RI_TYPE_MASK | HID_RI_TAG_MASK))
		{
			case HID_RI_PUSH(0):
				if (CurrStateTable == &StateTable[HID_STATETABLE_STACK_DEPTH - 1])
				  return HID_PARSE_HIDStackOverflow;

				memcpy((CurrStateTable + 1),
				       CurrStateTable,
				       sizeof(HID_ReportItem_t));

				CurrStateTable++;
				break;

			case HID_RI_POP(0):
				if (CurrStateTable == &StateTable[0])
				  return HID_PARSE_HIDStackUnderflow;

				CurrStateTable--;
				break;

			case HID_RI_USAGE_PAGE(0):
				if ((HIDReportItem & HID_RI_DATA_SIZE_MASK) == HID_RI_DATA_BITS_32)
				  CurrStateTable->Attributes.Usage.Page = (ReportItemData >> 16);

				CurrStateTable->Attributes.Usage.Page       = ReportItemData;
				break;

			case HID_RI_LOGICAL_MINIMUM(0):
				CurrStateTable->Attributes.Logical.Minimum  = ReportItemData;
				break;

			case HID_RI_LOGICAL_MAXIMUM(0):
				CurrStateTable->Attributes.Logical.Maximum  = ReportItemData;
				break;

			case HID_RI_PHYSICAL_MINIMUM(0):
				CurrStateTable->Attributes.Physical.Minimum = ReportItemData;
				break;

			case HID_RI_PHYSICAL_MAXIMUM(0):
				CurrStateTable->Attributes.Physical.Maximum = ReportItemData;
				break;

			case HID_RI_UNIT_EXPONENT(0):
				CurrStateTable->Attributes.Unit.Exponent    = ReportItemData;
				break;

			case HID_RI_UNIT(0):
				CurrStateTable->Attributes.Unit.Type        = ReportItemData;
				break;

			case HID_RI_REPORT_SIZE(0):
				CurrStateTable->Attributes.BitSize          = ReportItemData;
				break;

			case HID_RI_REPORT_COUNT(0):
				CurrStateTable->ReportCount                 = ReportItemData;
				break;

			case HID_RI_REPORT_ID(0):
				CurrStateTable->ReportID                    = ReportItemData;

				if (ParserData->UsingReportIDs)
				{
					CurrReportIDInfo = NULL;

					for (i = 0; i < ParserData->TotalDeviceReports; i++)
					{
						if (ParserData->ReportIDSizes[i].ReportID == CurrStateTable->ReportID)
						{
							CurrReportIDInfo = &ParserData->ReportIDSizes[i];
							break;
						}
					}

					if (CurrReportIDInfo == NULL)
					{
						if (ParserData->TotalDeviceReports == HID_MAX_REPORT_IDS)
						  return HID_PARSE_InsufficientReportIDItems;

						CurrReportIDInfo = &ParserData->ReportIDSizes[ParserData->TotalDeviceReports++];
						memset(CurrReportIDInfo, 0x00, sizeof(HID_ReportSizeInfo_t));
					}
				}

				ParserData->UsingReportIDs = 1;

				CurrReportIDInfo->ReportID = CurrStateTable->ReportID;
				break;

			case HID_RI_USAGE(0):
				if (UsageListSize == HID_USAGE_STACK_DEPTH)
				  return HID_PARSE_UsageListOverflow;

				UsageList[UsageListSize++] = ReportItemData;
				break;

			case HID_RI_USAGE_MINIMUM(0):
				UsageMinMax.Minimum = ReportItemData;
				break;

			case HID_RI_USAGE_MAXIMUM(0):
				UsageMinMax.Maximum = ReportItemData;
				break;

			case HID_RI_COLLECTION(0):
				if (CurrCollectionPath == NULL)
				{
					CurrCollectionPath = &ParserData->CollectionPaths[0];
				}
				else
				{
					HID_CollectionPath_t* ParentCollectionPath = CurrCollectionPath;

					CurrCollectionPath = &ParserData->CollectionPaths[1];

					while (CurrCollectionPath->Parent != NULL)
					{
						if (CurrCollectionPath == &ParserData->CollectionPaths[HID_MAX_COLLECTIONS - 1])
						  return HID_PARSE_InsufficientCollectionPaths;

						CurrCollectionPath++;
					}

					CurrCollectionPath->Parent = ParentCollectionPath;
				}

				CurrCollectionPath->Type       = ReportItemData;
				CurrCollectionPath->Usage.Page = CurrStateTable->Attributes.Usage.Page;

				if (UsageListSize)
				{
					CurrCollectionPath->Usage.Usage = UsageList[0];

					for (i = 1; i < UsageListSize; i++)
					  UsageList[i - 1] = UsageList[i];

					UsageListSize--;
				}
				else if (UsageMinMax.Minimum <= UsageMinMax.Maximum)
				{
					CurrCollectionPath->Usage.Usage = UsageMinMax.Minimum++;
				}

				break;

			case HID_RI_END_COLLECTION(0):
				if (CurrCollectionPath == NULL)
				  return HID_PARSE_UnexpectedEndCollection;

				CurrCollectionPath = CurrCollectionPath->Parent;
				break;

			case HID_RI_INPUT(0):
			case HID_RI_OUTPUT(0):
			case HID_RI_FEATURE(0):
				for (ReportItemNum = 0; ReportItemNum < CurrStateTable->ReportCount; ReportItemNum++)
				{
					HID_ReportItem_t NewReportItem;

					memcpy(&NewReportItem.Attributes,
					       &CurrStateTable->Attributes,
					       sizeof(HID_ReportItem_Attributes_t));

					NewReportItem.ItemFlags      = ReportItemData;
					NewReportItem.CollectionPath = CurrCollectionPath;
					NewReportItem.ReportID       = CurrStateTable->ReportID;

					if (UsageListSize)
					{
						NewReportItem.Attributes.Usage.Usage = UsageList[0];

						for (i = 1; i < UsageListSize; i++)
						  UsageList[i - 1] = UsageList[i];

						UsageListSize--;
					}
					else if (UsageMinMax.Minimum <= UsageMinMax.Maximum)
					{
						NewReportItem.Attributes.Usage.Usage = UsageMinMax.Minimum++;
					}

					uint8_t ItemTypeTag = (HIDReportItem & (HID_RI_TYPE_MASK | HID_RI_TAG_MASK));

					if (ItemTypeTag == HID_RI_INPUT(0))
					  NewReportItem.ItemType = HID_REPORT_ITEM_In;
					else if (ItemTypeTag == HID_RI_OUTPUT(0))
					  NewReportItem.ItemType = HID_REPORT_ITEM_Out;
					else
					  NewReportItem.ItemType = HID_REPORT_ITEM_Feature;

					NewReportItem.BitOffset = CurrReportIDInfo->ReportSizeBits[NewReportItem.ItemType];

					CurrReportIDInfo->ReportSizeBits[NewReportItem.ItemType] += CurrStateTable->Attributes.BitSize;

					ParserData->LargestReportSizeBits = MAX(ParserData->LargestReportSizeBits, CurrReportIDInfo->ReportSizeBits[NewReportItem.ItemType]);

					if (ParserData->TotalReportItems == HID_MAX_REPORTITEMS)
					  return HID_PARSE_InsufficientReportItems;

					memcpy(&ParserData->ReportItems[ParserData->TotalReportItems],
					       &NewReportItem, sizeof(HID_ReportItem_t));

					if (!(ReportItemData & HID_IOF_CONSTANT) && CALLBACK_HIDParser_FilterHIDReportItem(&NewReportItem))
					  ParserData->TotalReportItems++;
				}

				break;

			default:
				break;
		}

		if ((HIDReportItem & HID_RI_TYPE_MASK) == HID_RI_TYPE_MAIN)
		{
			UsageMinMax.Minimum = 0;
			UsageMinMax.Maximum = 0;
			UsageListSize       = 0;
		}
	}

	if (!(ParserData->TotalReportItems))
	  return HID_PARSE_NoUnfilteredReportItems;
    
	return HID_PARSE_Successful;
}

uint8_t USB_GetHIDReportItemInfo(const uint8_t* ReportData,
                              HID_ReportItem_t* const ReportItem)
{
	if (ReportItem == NULL)
	  return 0;

	uint16_t DataBitsRem  = ReportItem->Attributes.BitSize;
	uint16_t CurrentBit   = ReportItem->BitOffset;
	uint32_t BitMask      = (1 << 0);

	if (ReportItem->ReportID)
	{
		if (ReportItem->ReportID != ReportData[0])
		  return 0;

		ReportData++;
	}

	ReportItem->PreviousValue = ReportItem->Value;
	ReportItem->Value = 0;

	while (DataBitsRem--)
	{
		if (ReportData[CurrentBit / 8] & (1 << (CurrentBit % 8)))
		  ReportItem->Value |= BitMask;

		CurrentBit++;
		BitMask <<= 1;
	}

	return 1;
}

void USB_SetHIDReportItemInfo(uint8_t* ReportData,
                              HID_ReportItem_t* const ReportItem)
{
	if (ReportItem == NULL)
	  return;

	uint16_t DataBitsRem  = ReportItem->Attributes.BitSize;
	uint16_t CurrentBit   = ReportItem->BitOffset;
	uint32_t BitMask      = (1 << 0);

	if (ReportItem->ReportID)
	{
		ReportData[0] = ReportItem->ReportID;
		ReportData++;
	}

	ReportItem->PreviousValue = ReportItem->Value;

	while (DataBitsRem--)
	{
		if (ReportItem->Value & BitMask)
		  ReportData[CurrentBit / 8] |= (1 << (CurrentBit % 8));

		CurrentBit++;
		BitMask <<= 1;
	}
}

uint16_t USB_GetHIDReportSize(HID_ReportInfo_t* const ParserData,
                              const uint8_t ReportID,
                              const uint8_t ReportType)
{
    uint8_t i;
    uint16_t ReportSizeBits;

	for (i = 0; i < HID_MAX_REPORT_IDS; i++)
	{
		ReportSizeBits = ParserData->ReportIDSizes[i].ReportSizeBits[ReportType];

		if (ParserData->ReportIDSizes[i].ReportID == ReportID)
		  return (ReportSizeBits / 8) + ((ReportSizeBits % 8) ? 1 : 0);
	}

	return 0;
}

/** Callback for the HID Report Parser. This function is called each time the HID report parser is about to store
 *  an IN, OUT or FEATURE item into the HIDReportInfo structure. To save on RAM, we are able to filter out items
 *  we aren't interested in (preventing us from being able to extract them later on, but saving on the RAM they would
 *  have occupied).
 *
 *  \param[in] CurrentItem  Pointer to the item the HID report parser is currently working with
 *
 *  \return Boolean \c true if the item should be stored into the HID report structure, \c false if it should be discarded
 */
uint8_t CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const CurrentItem)
{
	uint8_t IsJoystick = 0;
    HID_CollectionPath_t* CurrPath;

	/* Iterate through the item's collection path, until either the root collection node or a collection with the
	 * Joystick Usage is found - this prevents Mice, which use identical descriptors except for the Joystick usage
	 * parent node, from being erroneously treated as a joystick by the demo
	 */
	for (CurrPath = CurrentItem->CollectionPath; CurrPath != NULL; CurrPath = CurrPath->Parent)
	{
		if ((CurrPath->Usage.Page  == USAGE_PAGE_GENERIC_DCTRL) &&
		    (CurrPath->Usage.Usage == USAGE_JOYSTICK))
		{
			IsJoystick = 1;
			break;
		}
	}

	/* If a collection with the joystick usage was not found, indicate that we are not interested in this item */
	if (!IsJoystick)
	  return 0;

	/* Check the attributes of the current item - see if we are interested in it or not;
	 * only store BUTTON and GENERIC_DESKTOP_CONTROL items into the Processed HID Report
	 * structure to save RAM and ignore the rest
	 */

    return (((CurrentItem->Attributes.Usage.Page == USAGE_PAGE_BUTTON) && (CurrentItem->Attributes.Usage.Usage < 5)) ||
            (CurrentItem->Attributes.Usage.Usage == USAGE_X) ||
            (CurrentItem->Attributes.Usage.Usage == USAGE_Y) ||
            (CurrentItem->Attributes.Usage.Usage == USAGE_Z) ||
            (CurrentItem->Attributes.Usage.Usage == USAGE_RZ) ||
            (CurrentItem->Attributes.Usage.Usage == USAGE_SLIDER));

}

