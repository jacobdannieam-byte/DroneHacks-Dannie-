from types import SimpleNamespace

from examples.hybrid_native_shadow import optional_message_info, transport_metadata


def test_humble_message_only_delivery_does_not_invent_rmw_metadata():
    rows = []
    message = object()
    callback = optional_message_info(lambda msg, info: rows.append(
        (msg, transport_metadata(info, callback_system_ns=100))))
    callback(message)
    assert rows == [(message, {'callback_system_ns': 100, 'message_info': 'unavailable',
                              'rmw_received_ns': None, 'rmw_source_ns': None,
                              'publication_sequence': None})]


def test_mapping_and_object_message_info_preserve_original_metadata():
    fields = {'received_timestamp': 123, 'source_timestamp': 111,
              'publication_sequence_number': 5}
    rows = []
    callback = optional_message_info(lambda msg, info: rows.append(
        transport_metadata(info, callback_system_ns=150)))
    callback(object(), fields)
    callback(object(), SimpleNamespace(**fields))
    assert rows[0] == rows[1] == {'callback_system_ns': 150, 'message_info': 'provided',
                                'rmw_received_ns': 123, 'rmw_source_ns': 111,
                                'publication_sequence': 5}


def test_partial_message_info_leaves_unavailable_fields_null():
    assert transport_metadata({}, callback_system_ns=100)['rmw_source_ns'] is None
