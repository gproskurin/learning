defmodule ElixirChunk do

@max_count 10
@max_size 100

@empty_acc {[], 0, 0} # {items, items_count, items_size}


def chunk_sqs(items, get_item_size_func) do
    Enum.chunk_while(
        items,
        @empty_acc,
        fn item, acc -> chunk_fun(item, acc, get_item_size_func) end,
        &after_fun/1
    )
end


defp chunk_fun(item, @empty_acc, get_item_size_func) do
    new_acc = {[item], 1, get_item_size_func.(item)}
    {:cont, new_acc}
end

defp chunk_fun(item, {chunk, len, _size}, get_item_size_func) when len >= @max_count do
    new_acc = {[item], 1, get_item_size_func.(item)}
    {:cont, :lists.reverse(chunk), new_acc}
end

defp chunk_fun(item, {chunk, len, size}, get_item_size_func) do
    item_sz = get_item_size_func.(item)
    total_sz = size + item_sz
    case total_sz > @max_size do
        true ->
            {:cont, :lists.reverse(chunk), {[item], 1, item_sz}}
        false ->
            {:cont, {[item | chunk], len+1, total_sz}}
    end
end


defp after_fun(@empty_acc = acc), do: {:cont, acc}
defp after_fun({chunk, _, _}), do: {:cont, :lists.reverse(chunk), @empty_acc}


end
