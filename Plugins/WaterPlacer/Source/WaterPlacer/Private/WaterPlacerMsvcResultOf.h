#pragma once

// MSVC's <ppltasks.h> still uses std::result_of. C++20 removed it, and UnrealEd
// pulls that header when this editor module is compiled as C++20 (Cesium span).
#if defined(_MSC_VER)
#include <type_traits>
#if defined(_HAS_CXX20) && _HAS_CXX20
namespace std
{
	template <class>
	struct result_of;
	template <class F, class... Args>
	struct result_of<F(Args...)> : invoke_result<F, Args...>
	{
	};
	template <class T>
	using result_of_t = typename result_of<T>::type;
}
#endif
#endif
