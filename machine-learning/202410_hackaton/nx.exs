#!/usr/bin/env elixir

Mix.install([
	:nx,
#	:exla
])
#Nx.global_default_backend(EXLA.Backend)


defmodule NxTest do


defp compute_model_output(x, w, b) do
	Nx.multiply(x, w) |> Nx.add(b)
end


defp compute_cost(x, y, w, b) do
	compute_model_output(x, w, b) |> Nx.subtract(y) |> Nx.pow(2) |> Nx.sum |> Nx.divide(2 * Nx.size(x))
end


defp compute_gradient(x, y, w, b) do
	dj_db = compute_model_output(x, w, b) |> Nx.subtract(y) |> Nx.divide(Nx.size(x))
	dj_dw = Nx.multiply(dj_db, x)
	{Nx.sum(dj_dw), Nx.sum(dj_db)}
end


@eps Nx.tensor(0.000001, type: :f64)
defp is_small(t) do
	r = Nx.less(Nx.abs(t), @eps) |> Nx.all
	case Nx.to_number(r) do
		0 -> :false
		1 -> :true
	end
end


defp gradient_descent(_x, _y, w_in, b_in, _alpha, {i_max, i_max}, _cost_function, _gradient_function) do
	:io.format("END_ITER~n")
	{w_in, b_in}
end
defp gradient_descent(x, y, w_in, b_in, alpha, {i, i_max}, cost_function, gradient_function) do
	{dj_dw, dj_db} = gradient_function.(x, y, w_in, b_in)
	w = Nx.subtract(
		w_in,
		Nx.multiply(alpha, dj_dw)
	)
	b = Nx.subtract(
		b_in,
		Nx.multiply(alpha, dj_db)
	)
	j = cost_function.(x, y, w, b)
	:io.format("GRAD_DESCENT_PROGRESS: i=#{inspect i} cost=#{inspect j} dj_dw=#{inspect dj_dw} dj_db=#{inspect dj_db} w=#{inspect w} b=#{inspect b}~n")
	case is_small(dj_dw) and is_small(dj_db) do
		:true ->
			:io.format("END_EPSILON~n")
			{w, b}
		:false ->
			gradient_descent(x, y, w, b, alpha, {i+1, i_max}, cost_function, gradient_function)
	end
end


def run() do
	xy_train = Nx.tensor(
		[
			[1, 300],
			[2, 500],
			#[7, 1893],
			#[3, 777],
			#[5, 1200],
		],
		type: :f64,
		names: [:x, :y]
	)
	:io.format("xy_train=#{inspect xy_train}~n")

	# TODO
	x_train = Nx.transpose(xy_train)[0]
	y_train = Nx.transpose(xy_train)[1]
	:io.format("x_train=#{inspect(x_train)}~n")
	:io.format("y_train=#{inspect(y_train)}~n")

	w = 0
	b = 0
	c = compute_model_output(x_train, w, b)
	:io.format("COMPUTE=#{inspect c}~n")

	cost = compute_cost(x_train, y_train, w, b)
	:io.format("COMPUTE_COST=#{inspect cost}~n")

	grad = compute_gradient(x_train, y_train, w, b)
	:io.format("COMPUTE_GRAD=#{inspect grad}~n")

	fn_compute_cost = fn (x,y,w,b) -> compute_cost(x,y,w,b) end
	fn_compute_gradient = fn (x,y,w,b) -> compute_gradient(x,y,w,b) end

	{tc, r} = :timer.tc(
		fn ->
			gradient_descent(x_train, y_train, w, b, 0.01, {0, 1000000}, fn_compute_cost, fn_compute_gradient)
		end
	)
	{w_end, b_end} = r
	:io.format("GRAD_DESCENT_END: time=#{inspect tc} w=#{inspect w_end} b=#{inspect b_end}~n")
end

end

NxTest.run()

