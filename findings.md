# Findings

- `_is_live_map_id()` treats empty text, configured live ID, and `DEFAULT_LIVE_MAP_ID` as the live source.
- `_load_map_registry_state()` currently accepts any record with a map ID and bundle directory, including stale `LIVE_MAP` entries.
- `_save_map_registry_state()` serializes the in-memory registry without filtering reserved live IDs.
- `handle_map_save_request()` checks the registry before checking whether the request ID is live, so a stale entry causes metadata-only success.
- Its new-map branch currently preserves the literal `LIVE_MAP` as `target_map_id`, which would create a reserved-ID bundle after only fixing the branch guard.
- `super_lio_mode_manager._handle_save_map()` correctly requires active `MAPPING` and switches to localization after a successful save.
- Therefore the APP must save before explicitly leaving mapping mode.
