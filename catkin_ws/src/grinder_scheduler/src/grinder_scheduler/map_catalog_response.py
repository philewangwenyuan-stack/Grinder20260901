def fill_map_catalog_response(
    response,
    entries,
    find_record_by_id,
    include_thumbnails=True,
    max_items=200,
    max_thumb_b64_total=512000,
):
    """
    Populate MapCatalogResponse items with a bounded payload size.

    Returns a tuple:
        (item_count, thumbnails_attached_count, thumbnails_dropped_count)
    """
    bounded_entries = list(entries)[: max(1, int(max_items))]
    thumb_budget = max(0, int(max_thumb_b64_total))
    thumb_used = 0
    thumbs_attached = 0
    thumbs_dropped = 0

    for name, map_id, _path, size_bytes, total_work_area_m2, estimated_time_s in bounded_entries:
        item = response.items.add()
        record = find_record_by_id(map_id)
        if hasattr(item, "map_id"):
            item.map_id = str(map_id or "")
        if hasattr(item, "name"):
            item.name = str(name)
        if hasattr(item, "size_bytes"):
            item.size_bytes = int(size_bytes)
        if hasattr(item, "saved_at"):
            item.saved_at = int(record.get("saved_at", 0) if isinstance(record, dict) else 0)
        if hasattr(item, "created_at"):
            created_at = ""
            if isinstance(record, dict):
                created_at = str(record.get("created_at", "") or "").strip()
                if (not created_at) and int(record.get("saved_at", 0) or 0) > 0:
                    import datetime

                    created_at = datetime.datetime.fromtimestamp(
                        int(record.get("saved_at", 0))
                    ).strftime("%Y-%m-%d %H:%M:%S")
            item.created_at = str(created_at)
        if hasattr(item, "total_work_area_m2"):
            item.total_work_area_m2 = float(max(0.0, total_work_area_m2))
        if hasattr(item, "estimated_time_s"):
            item.estimated_time_s = float(estimated_time_s)

        if not isinstance(record, dict):
            continue
        if hasattr(item, "thumbnail_format"):
            item.thumbnail_format = str(record.get("thumb_format", "") or "")
        if hasattr(item, "thumbnail_width"):
            item.thumbnail_width = int(max(0, int(record.get("thumb_width", 0) or 0)))
        if hasattr(item, "thumbnail_height"):
            item.thumbnail_height = int(max(0, int(record.get("thumb_height", 0) or 0)))
        if not hasattr(item, "thumbnail_image_b64"):
            continue
        if not include_thumbnails:
            item.thumbnail_image_b64 = ""
            thumbs_dropped += 1
            continue

        thumb_b64 = str(record.get("thumb_b64", "") or "")
        next_size = len(thumb_b64.encode("utf-8")) if thumb_b64 else 0
        if thumb_b64 and (thumb_used + next_size) <= thumb_budget:
            item.thumbnail_image_b64 = thumb_b64
            thumb_used += next_size
            thumbs_attached += 1
        else:
            item.thumbnail_image_b64 = ""
            thumbs_dropped += 1

    return len(bounded_entries), thumbs_attached, thumbs_dropped
