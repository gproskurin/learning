defmodule ElixirChunkTest do
  use ExUnit.Case
  alias ElixirChunk, as: Ec

  test "empty" do
    assert [] == Ec.chunk_sqs([], fn _ -> 0 end)
  end

  test "len -1" do
    assert [[1,2,3,4,5,6,7,8,9]] == Ec.chunk_sqs([1,2,3,4,5,6,7,8,9], fn _ -> 0 end)
  end

  test "len eq" do
    assert [[1,2,3,4,5,6,7,8,9,10]] == Ec.chunk_sqs([1,2,3,4,5,6,7,8,9,10], fn _ -> 0 end)
  end

  test "len +1" do
    assert [[1,2,3,4,5,6,7,8,9,10],[11]] == Ec.chunk_sqs([1,2,3,4,5,6,7,8,9,10,11], fn _ -> 0 end)
  end

  test "len +2" do
    assert [[1,2,3,4,5,6,7,8,9,10],[11,12]] == Ec.chunk_sqs([1,2,3,4,5,6,7,8,9,10,11,12], fn _ -> 0 end)
  end

  test "at least 1" do
    assert [[1000]] == Ec.chunk_sqs([1000], fn x -> x end)
  end

  test "size eq" do
    assert [[50,50]] == Ec.chunk_sqs([50,50], fn x -> x end)
  end

  test "size gt" do
    assert [[51],[50]] == Ec.chunk_sqs([51,50], fn x -> x end)
  end

  test "size gt2" do
    assert [[1000],[1000]] == Ec.chunk_sqs([1000,1000], fn x -> x end)
  end

end
