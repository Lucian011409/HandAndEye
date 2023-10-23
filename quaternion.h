

/*

这个文件是在mist库的quaternion的基础上进行修改的。

这里主要将扩展quaternion，euler以及旋转矩阵之间的变换


*/



// 
// Copyright (c) 2003-2006, MIST Project, Intelligent Media Integration COE, Nagoya University
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 
// 3. Neither the name of the Nagoya University nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
// IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 

/// @file mist/quaternion.h
//!
//! @brief クォータニオン（四元数）を扱うためのライブラリ
//!

#pragma once
#include <limits>

#include <mist/matrix.h>
using namespace mist;


#ifndef __INCLUDE_MIST_CONF_H__
#include <mist/config/mist_conf.h>
#endif

#ifndef __INCLUDE_MIST_TYPE_TRAIT_H__
#include <mist/config/type_trait.h>
#endif

#ifndef __INCLUDE_MIST_VECTOR__
#include <mist/vector.h>
#endif


#include <cmath>

// mist名前空間の始まり



/// @brief クォータニオン（四元数）を扱うクラス
//! 
//! @param T … 内部で用いるデータ型     
//! 
template < class T >
class Quaternion
{
public:
	typedef T value_type;					///< @brief MISTのコンテナ内に格納するデータ型．mist::array< data > の data と同じ
	typedef size_t size_type;				///< @brief 符号なしの整数を表す型．コンテナ内の要素数や，各要素を指定するときなどに利用し，内部的には size_t 型と同じ
	typedef ptrdiff_t difference_type;		///< @brief 符号付きの整数を表す型．コンテナ内の要素数や，各要素を指定するときなどに利用し，内部的には ptrdiff_t 型と同じ

public:
	value_type w;		///< @brief 実数成分
	value_type x;		///< @brief 虚数成分1
	value_type y;		///< @brief 虚数成分2
	value_type z;		///< @brief 虚数成分3

	/// @brief デフォルトコンストラクタ（全要素を0で初期化する）
	Quaternion( ) : w( 0 ), x( 0 ), y( 0 ), z( 0 ){ }

	Quaternion(const vector3< T > &v): w( 0 ), x( static_cast< value_type >( v.x ) ), y( static_cast< value_type >( v.y ) ), z( static_cast< value_type >( v.z ) ){ }

	/// @brief ww，xx，yy，zz の値を用いて初期化する
	Quaternion( const value_type &ww, const value_type &xx, const value_type &yy, const value_type &zz ) : w( ww ), x( xx ), y( yy ), z( zz ){ }

	/// @brief ww，xx，yy，zz の値を用いて初期化する
	explicit Quaternion( const value_type &ww ) : w( ww ), x( 0 ), y( 0 ), z( 0 ){ }

	/// @brief 他のクォータニオンオブジェクト（データ型が異なる）を用いて初期化する
	template < class TT >
	Quaternion( const Quaternion< TT > &q ) : w( static_cast< value_type >( q.w ) ), x( static_cast< value_type >( q.x ) ), y( static_cast< value_type >( q.y ) ), z( static_cast< value_type >( q.z ) ){ }

	/// @brief 他のクォータニオンオブジェクト（データ型が同じ）を用いて初期化する
	Quaternion( const Quaternion< T > &q ) : w( q.w ), x( q.x ), y( q.y ), z( q.z ){ }


	/// @brief 実数成分 ww と虚数成分のベクトル v を用いて初期化する
	template < class TT >
	Quaternion( value_type ww, const vector3< TT > &v ) : w( ww ), x( static_cast< value_type >( v.x ) ), y( static_cast< value_type >( v.y ) ), z( static_cast< value_type >( v.z ) ){ }


	/// @brief クォータニオンを用いた任意軸周りの回転
	//! 
	//! @attention 右手系の場合は右ねじ回転，左手系の場合は左ねじ回転となるので注意！！      //右手系时右螺丝回转，左手系的场合左螺丝回转的注意！！
	//! @attention 回転角度の単位は度を用いる（ラジアンではないので注意！！）                //旋转角度的单位是次使用（弧度，因为不注意！！）   ???
	//! 
	//! @param[in] axis  … 回転軸
	//! @param[in] theta … 回転角度
	//! 
	template < class TT >
	Quaternion( const vector3< TT > &axis, value_type theta )
	{
		double t = theta * 3.1415926535897932384626433832795 / 180.0 / 2.0;
		double c = std::cos( t );
		double s = std::sin( t );
		w = static_cast< value_type >( c );
		x = static_cast< value_type >( s * axis.x );
		y = static_cast< value_type >( s * axis.y );
		z = static_cast< value_type >( s * axis.z );
	}


	/// @brief カメラの視線方向と上向き方向を用いて，カメラの姿勢を表すクォータニオンを計算する     
	//Using line-of-sight direction and the upward direction of the camera, calculating the quaternion representing the posture of the camera    クォータニオン：Quaternion
	//! 
	//! @param[in] dir … 回転前のベクトル   //旋转前的向量
	//! @param[in] up  … 回転後のベクトル   //旋转后的向量
	//! 
	template < class TT >
	Quaternion( vector3< TT > dir, vector3< TT > up )
	{
		// ワールド座標の単位ベクトル  //世界坐标的单位向量
		vector3< TT > e2( 0, 1, 0 );
		vector3< TT > e3( 0, 0, 1 );

		// 単位ベクトルにする  //单位化
		dir = dir.unit( );
		up  = up.unit( );

		// 視線方向を合わせるクォータニオンを作成   //Create a quaternion to adjust the line-of-sight direction
		Quaternion q1 = Quaternion::rotate( e3, dir );

		// Y軸を回転させる    //rotate the Y-axis
		e2 = q1.rotate( e2 );

		// 上向き方向を合わせるクォータニオンを作成  //Create a quaternion to match the upward direction
		Quaternion q2 = Quaternion::rotate( e2, up, dir );

		// 回転を合成する  //synthesize the rotation
		operator =( q2 * q1 );
		
	}


	/// @brief 他のクォータニオンオブジェクトを代入する
	template < class TT >
	const Quaternion &operator =( const Quaternion< TT > &q )
	{
		w = static_cast< value_type >( q.w );
		x = static_cast< value_type >( q.x );
		y = static_cast< value_type >( q.y );
		z = static_cast< value_type >( q.z );
		return ( *this );
	}

	/// @brief 他のクォータニオンオブジェクトを代入する
	const Quaternion &operator =( const Quaternion< T > &q )
	{
		if( &q != this )
		{
			w = q.w;
			x = q.x;
			y = q.y;
			z = q.z;
		}
		return ( *this );
	}

	/// @brief 符号反転したクォータニオンを返す
	Quaternion operator -( ) const { return ( Quaternion( -w, -x, -y, -z ) ); }

	/// @brief クォータニオンの足し算
	//! 
	//! \f[ \mbox{\boldmath p} + \mbox{\boldmath q} = \left( p_w + q_w \;,\; p_x + q_x \;,\; p_y + q_y \;,\; p_z + q_z \right)^T \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @return 足し算結果
	//! 
	template< class TT >
	const Quaternion &operator +=( const Quaternion< TT > &q )
	{
		w = static_cast< value_type >( w + q.w );
		x = static_cast< value_type >( x + q.x );
		y = static_cast< value_type >( y + q.y );
		z = static_cast< value_type >( z + q.z );
		return( *this );
	}

	/// @brief クォータニオンへの実数成分の足し算
	//! 
	//! \f[ \mbox{\boldmath p} + a = \left( p_w + a \;,\; p_x \;,\; p_y \;,\; p_z \right)^T \f]
	//! 
	//! @param[in] a … 実数成分
	//! 
	//! @return 足し算結果
	//! 
#if defined( __MIST_MSVC__ ) && __MIST_MSVC__ < 7
	const Quaternion &operator +=( const double &a )
#else
	template < class TT >
	const Quaternion &operator +=( const TT &a )
#endif
	{
		w = static_cast< value_type >( w + a );
		return( *this );
	}

	/// @brief クォータニオンの引き算
	//! 
	//! \f[ \mbox{\boldmath p} - \mbox{\boldmath q} = \left( p_w - q_w \;,\; p_x - q_x \;,\; p_y - q_y \;,\; p_z - q_z \right)^T \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @return 引き算結果
	//! 
	template< class TT >
	const Quaternion &operator -=( const Quaternion< TT > &q )
	{
		w = static_cast< value_type >( w - q.w );
		x = static_cast< value_type >( x - q.x );
		y = static_cast< value_type >( y - q.y );
		z = static_cast< value_type >( z - q.z );
		return( *this );
	}

	/// @brief クォータニオンへの実数成分の引き算
	//! 
	//! \f[ \mbox{\boldmath p} - a = \left( p_w - a \;,\; p_x \;,\; p_y \;,\; p_z \right)^T \f]
	//! 
	//! @param[in] a … 実数成分
	//! 
	//! @return 引き算結果
	//! 
#if defined( __MIST_MSVC__ ) && __MIST_MSVC__ < 7
	const Quaternion &operator -=( const double &a )
#else
	template < class TT >
	const Quaternion &operator -=( const TT &a )
#endif
	{
		w = static_cast< value_type >( w - a );
		return( *this );
	}

	/** @brief クォータニオンの掛け算    Quaternion Multiplication
	*  
	*  \f[
	*      \mbox{\boldmath p} \times \mbox{\boldmath q} =
	*          \left(
	*             p_w \times q_w - p_x \times q_x - p_y \times q_y - p_z \times q_z \;,\;
	*             p_w \times q_x + p_x \times q_w + p_y \times q_z - p_z \times q_y \;,\;
	*             p_w \times q_y + p_y \times q_w + p_z \times q_x - p_x \times q_z \;,\;
	*             p_w \times q_z + p_z \times q_w + p_x \times q_y - p_y \times q_x
	*          \right)^T
	*  \f]
	*  
	*  @param[in] q … 右辺値
	*  
	*  @return 掛け算結果
	*/
	template < class TT >
	const Quaternion &operator *=( const Quaternion< TT > &q )
	{
		value_type ww = static_cast< value_type >( w * q.w - x * q.x - y * q.y - z * q.z );
		value_type xx = static_cast< value_type >( w * q.x + x * q.w + y * q.z - z * q.y );
		value_type yy = static_cast< value_type >( w * q.y + y * q.w + z * q.x - x * q.z );
		value_type zz = static_cast< value_type >( w * q.z + z * q.w + x * q.y - y * q.x );
		w = ww;
		x = xx;
		y = yy;
		z = zz;
		return( *this );
	}

	/// @brief クォータニオンへの実数成分の掛け算
	//! 
	//! \f[ \mbox{\boldmath p} \times a = \left( p_w \times a \;,\; p_x \times a \;,\; p_y \times a \;,\; p_z \times a \right)^T \f]
	//! 
	//! @param[in] a … 実数成分
	//! 
	//! @return 掛け算結果
	//! 
#if defined( __MIST_MSVC__ ) && __MIST_MSVC__ < 7
	const Quaternion &operator *=( const double &a )
#else
	template < class TT >
	const Quaternion &operator *=( const TT &a )
#endif
	{
		w = static_cast< value_type >( w * a );
		x = static_cast< value_type >( x * a );
		y = static_cast< value_type >( y * a );
		z = static_cast< value_type >( z * a );
		return( *this );
	}

	/// @brief クォータニオンの割り算
	//! 
	//! \f[ \frac{ \mbox{\boldmath p} }{ \mbox{\boldmath q} } = \mbox{\boldmath p} \times \mbox{\boldmath q}^{-1} \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @return 掛け算結果
	//! 
	template < class TT >
	const Quaternion &operator /=( const Quaternion< TT > &q )
	{
		return( this->operator *=( q.inv( ) ) );
	}

	/// @brief クォータニオンを実数成分で割る
	//! 
	//! \f[ \mbox{\boldmath p} \div a = \left( p_w \div a \;,\; p_x \div a \;,\; p_y \div a \;,\; p_z \div a \right)^T \f]
	//! 
	//! @param[in] a … 実数成分
	//! 
	//! @return 掛け算結果
	//! 
#if defined( __MIST_MSVC__ ) && __MIST_MSVC__ < 7
	const Quaternion &operator /=( const double &a )
#else
	template < class TT >
	const Quaternion &operator /=( const TT &a )
#endif
	{
		w = static_cast< value_type >( w / a );
		x = static_cast< value_type >( x / a );
		y = static_cast< value_type >( y / a );
		z = static_cast< value_type >( z / a );
		return( *this );
	}

	operator vector3<T>()
	{
		return vector3<T>(x,y,z);
	}

	/// @brief 2つのクォータニオンが同一かどうかを判定する
	//! 
	//! \f[ \mbox{\boldmath p} == \mbox{\boldmath q} \rightarrow p_w == q_w \; \wedge \; p_x == q_x \; \wedge \; p_y == q_y \; \wedge \; p_z == q_z \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @retval true  … 全ての要素が等しい場合
	//! @retval false … どれか1つでも等しくない場合
	//! 
	bool operator ==( const Quaternion &q ) const { return( w == q.w && x == q.x && y == q.y && z == q.z ); }

	/// @brief 2つのクォータニオンが等しくないどうかを判定する
	//! 
	//! \f[ \mbox{\boldmath p} \neq \mbox{\boldmath q} \rightarrow \overline{ p_w = q_w \; \wedge \; p_x = q_x \; \wedge \; p_y = q_y \; \wedge \; p_z = q_z } \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @retval true  … どれか1つでも等しくない場合
	//! @retval false … 全ての要素が等しい場合
	//! 
	bool operator !=( const Quaternion &q ) const { return( !( *this == q ) ); }

	/// @brief 2つのベクトルの < を判定する
	//! 
	//! \f[ \mbox{\boldmath p} < \mbox{\boldmath q} \rightarrow \overline{ p_w \ge q_w \; \wedge \; p_x \ge q_x \; \wedge \; p_y \ge q_y \; \wedge \; p_z \ge q_z } \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @retval true  … p <  q の場合
	//! @retval false … p >= q の場合
	//! 
	bool operator < ( const Quaternion &q ) const
	{
		if( w == q.w )
		{
			if( x == q.x )
			{
				if( y == q.y )
				{
					return( z < q.z );
				}
				else
				{
					return( y < q.y );
				}
			}
			else
			{
				return( x < q.x );
			}
		}
		else
		{
			return( w < q.w );
		}
	}

	/// @brief 2つのベクトルの <= を判定する
	//! 
	//! \f[ \mbox{\boldmath p} \le \mbox{\boldmath q} \rightarrow p_w \le q_w \; \wedge \; p_x \le q_x \; \wedge \; p_y \le q_y \; \wedge \; p_z \le q_z \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @retval true  … p <= q の場合
	//! @retval false … p >  q の場合
	//! 
	bool operator <=( const Quaternion &q ) const { return( q >= *this ); }

	/// @brief 2つのベクトルの > を判定する
	//! 
	//! \f[ \mbox{\boldmath p} > \mbox{\boldmath q} \rightarrow \overline{ p_w \le q_w \; \wedge \; p_x \le q_x \; \wedge \; p_y \le q_y \; \wedge \; p_z \le q_z } \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @retval true  … p >  q の場合
	//! @retval false … p <= q の場合
	//! 
	bool operator > ( const Quaternion &q ) const { return( q < *this ); }

	/// @brief 2つのベクトルの >= を判定する
	//! 
	//! \f[ \mbox{\boldmath p} \ge \mbox{\boldmath q} \rightarrow p_w \ge q_w \; \wedge \; p_x \ge q_x \; \wedge \; p_y \ge q_y \; \wedge \; p_z \ge q_z \f]
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! @retval true  … p >= q の場合
	//! @retval false … p <  q の場合
	//! 
	bool operator >=( const Quaternion &q ) const { return( !( *this < q ) ); }


public:	// その他の関数

	/// @brief 共役クォータニオン
	//! 
	//! \f[ \overline{ \mbox{\boldmath p} } = \left( p_w \;,\; -p_x \;,\; -p_y \;,\; -p_z \right)^T \f]
	//! 
	const Quaternion conjugate( ) const 
	{
		return( Quaternion( w, -x, -y, -z ) );
	}

	/// @brief 逆クォータニオン
	//! 
	//! \f[ \mbox{\boldmath p}^{-1} = \frac{ \overline{ \mbox{\boldmath p} } }{ \left\| \mbox{\boldmath p} \right\|^2 } \f]
	//! 
	const Quaternion< double > inv( ) const
	{
		double length_ = length( );
		return( conjugate( ) / ( length_ * length_ ) );
	}

	/// @brief 単位クォータニオン
	//! 
	//! \f[ \frac{ \mbox{\boldmath p} }{ \left\| \mbox{\boldmath p} \right\|^2 } \f]
	//! 
	const Quaternion< double > unit( ) const
	{
		double length_ = length( );
		return( Quaternion< double >( w / length_, x / length_, y / length_, z / length_ ) );
	}

	/// @brief クォータニオンの内積
	//! 
	//! @param[in] q … 右辺値
	//! 
	//! \f[ p_w \times q_w + p_x \times q_x + p_y \times q_y + p_z \times q_z \f]
	//! 
	template < class TT >
	double inner( const Quaternion< TT > &q ) const
	{
		return( w * q.w + x * q.x + y * q.y + z * q.z );
	}

	/// @brief クォータニオンのノルム
	//! 
	//! \f[ \left\| \mbox{\boldmath p} \right\| = \sqrt{ p_w^2 + p_x^2 + p_y^2 + p_z^2 } \f]
	//! 
	double length( ) const { return (  std::sqrt( static_cast< double >( w * w + x * x + y * y + z * z ) ) ); }


	/// @brief クォータニオンを用いたベクトルの回転
	//! 
	//! @param[in] v … 回転されるベクトル
	//! 
	//! @return 回転後のベクトル
	//! 
	template < class TT >
	const vector3< TT > rotate( const vector3< TT > &v ) const
	{
		Quaternion q = ( *this ) * Quaternion( 0, static_cast< value_type >( v.x ), static_cast< value_type >( v.y ), static_cast< value_type >( v.z ) ) * inv( );
		return( vector3< TT >( static_cast< TT >( q.x ), static_cast< TT >( q.y ), static_cast< TT >( q.z ) ) );
	}



	/// @brief ベクトル1からベクトル2への回転を表すクォータニオンを作成する
	//! 
	//! @param[in] v1 … 回転前のベクトル
	//! @param[in] v2 … 回転後のベクトル
	//! 
	//! @return 回転を表すクォータニオン
	//! 
	template < class TT >
	static Quaternion rotate( vector3< TT > v1, vector3< TT > v2 )
	{
		// 単位ベクトルにする
		v1 = v1.unit( );
		v2 = v2.unit( );

		// 回転角度を計算する
		double dot = v1.inner( v2 );
		if( dot < -1.0 )
		{
			return( Quaternion( -1, 0, 0, 0 ) );
		}

		double c = std::sqrt( ( dot + 1.0 ) * 0.5 );

		if( std::abs( c - 1.0 ) < 1.0e-6 || c > 1.0 )
		{
			return( Quaternion( 1, 0, 0, 0 ) );
		}
		else if( std::abs( c + 1.0 ) < 1.0e-6 || c < -1.0 )
		{
			return( Quaternion( -1, 0, 0, 0 ) );
		}

		return( Quaternion( c, std::sqrt( 1.0 - c * c ) * v1.outer( v2 ).unit( ) ) );
	}

	/// @brief 指定した回転軸を用いてベクトル1からベクトル2への回転を表すクォータニオンを作成する
	//! 
	//! @param[in] v1   … 回転前のベクトル
	//! @param[in] v2   … 回転後のベクトル
	//! @param[in] axis … 回転軸ベクトル
	//! 
	//! @return 回転を表すクォータニオン
	//! 
	template < class TT >
	static Quaternion rotate( vector3< TT > v1, vector3< TT > v2, const vector3< TT > &axis )
	{
		// 単位ベクトルにする
		v1 = v1.unit( );
		v2 = v2.unit( );

		// 回転角度を計算する
		double dot = v1.inner( v2 );
		if( dot < -1.0 )
		{
			return( Quaternion( -1, 0, 0, 0 ) );
		}

		double c = std::sqrt( ( dot + 1.0 ) * 0.5 );

		if( std::abs( c - 1.0 ) < 1.0e-6 || c > 1.0 )
		{
			return( Quaternion( 1, 0, 0, 0 ) );
		}
		else if( std::abs( c + 1.0 ) < 1.0e-6 || c < -1.0 )
		{
			return( Quaternion( -1, 0, 0, 0 ) );
		}

		double s = std::sqrt( 1.0 - c * c );

		if( axis.inner( v1.outer( v2 ) ) < 0.0 )
		{
			s = -s;
		}

		return( Quaternion( c, s * axis ) );
	}

/************************************************************************/
/*
以下是为了matrix,quaternion,euler之间转换添加的一些成员
*/
/************************************************************************/


public:


	//这个算法和matlab中提供的算法有点不一致,是matlab中的转置
	//是否需要转置？？？？？？？？？？
	matrix<T> ConvertToMatrix()
	{
		matrix<T> mat(4, 4);
		mat(0, 0) = w*w + x*x - y*y - z*z;
		mat(0, 1) = 2 * (x*y - w*z);
		mat(0, 2) = 2 * (x*z + w*y);
		mat(0, 3) = 0;
		mat(1, 0) = 2 * (x*y + w*z);
		mat(1, 1) = w*w - x*x + y*y - z*z;
		mat(1, 2) = 2 * (y*z - w*x);
		mat(1, 3) = 0;
		mat(2, 0) = 2 * (x*z - w*y);
		mat(2, 1) = 2 * (y*z + w*x);
		mat(2, 2) = w*w - x*x - y*y + z*z;
		mat(2, 3) = 0;
		mat(3, 0) = 0;
		mat(3, 1) = 0;
		mat(3, 2) = 0;
		mat(3, 3) = 1;
		return(mat);

	}


	void ConvertFromMatrix(matrix<T> mat)
	{   
		
		if ((mat.rows() <3) || (mat.rows()> 4) || (mat.cols() <3) || (mat.rows() > 4)) 
		{
			w=x=y=z=0;
			return ;
		}

		//算法来源：http://www.redwiki.net/wiki/wiki/The%20Matrix%20and%20Quaternions%20FAQ
		//http://www.j3d.org/matrix_faq/matrfaq_latest.html
		//链接中的 The Matrix and Quaternions FAQ
		//代码参考：http://www.koders.com/cpp/fid3D86CD2B7AE5077ABF2A96AC578686B2FD95C847.aspx ...v\vnet\vnet\vnet\client\3d\  
		
		//这个算法和matlab中提供的算法也一致
		
		T	trace;
		T	s;

		trace = 1 + mat(0,0) + mat(1,1) + mat(2,2);//4*w*w
		if (trace > 1 )
		{
			s = sqrt(trace) * 2.0f; //0.5 / sqrt(trace);
			w= s * 0.25f; //(0.25 / s);
			x=((mat(2,1) - mat(1,2)) / s);
			y=((mat(0,2) - mat(2,0)) / s);
			z=((mat(1,0) - mat(0,1)) / s);
		}
		else
		{
			if ((mat(0,0) > mat(1,1)) && (mat(0,0) > mat(2,2)))
			{
				s = 2 * sqrt(1.0 + mat(0,0) - mat(1,1) - mat(2,2)); // s=4*x
				w=(mat(2,1) - mat(1,2)) / s;
				x=0.25 * s;
				y=(mat(1,0) + mat(0,1)) / s;
				z=(mat(0,2) + mat(2,0)) / s;
			}
			else if (mat(1,1) > mat(2,2))
			{
				s = 2 * sqrt(1.0 + mat(1,1) - mat(0,0) - mat(2,2)); // s=4*y
				w=(mat(0,2) - mat(2,0)) / s;
				x=(mat(1,0) + mat(0,1)) / s;
				y=0.25 * s;
				z=(mat(2,1) + mat(1,2)) / s;
			}
			else
			{
				s = 2 * sqrt(1.0 + mat(2,2) - mat(0,0) - mat(1,1)); // s=4*z
				w=(mat(1,0) - mat(0,1)) / s;
				x=(mat(0,2) + mat(2,0)) / s;
				y=(mat(2,1) + mat(1,2)) / s;
				z=0.25 * s;
			}
		}


		/*
		w = sqrt(1+mat(0,0)+mat(1,1)+mat(2,2))/2;

		x = -(mat(1,2) - mat(2,1)) / (4 * w);
		y = -(mat(2,0) - mat(0,2)) / (4 * w);
		z = -(mat(0,1) - mat(1,0)) / (4 * w);
		*/

	}


	void ConvertToEuler(T& Roll, T& Pitch, T& Yaw)
	{
		//Roll	= atan2(2*x*w + 2*y*z , 1 - 2*x*x - 2*y*y);	//[-PAI,PAI]
		//Pitch	= asin( 2*y*w - 2*x*z) ;					//[-PAI/2,PAI/2] 
		//Yaw		= atan2(2*x*y + 2*w*z , 1 - 2*y*y - 2*z*z);	//[-PAI,PAI]
	
		
		//matlab中的算法
		Roll	= atan2(2*x*w + 2*y*z , w*w - x*x - y*y + z*z);	//[-PAI,PAI]
		Pitch	= asin( 2*y*w - 2*x*z) ;					//[-PAI/2,PAI/2] 
		Yaw		= atan2(2*x*y + 2*w*z , w*w + x*x - y*y - z*z);	//[-PAI,PAI]
	
	}




	void ConvertFromEuler(const T roll, const T pitch, const T yaw)
	{
		//matlab中的算法
		w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
		x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
		y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
		z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
		if (w<0) 
		{
			w = -w;
			x = -x;
			y = -y;
			z = -z;
		}

		////chunfeng的共享空间 四元数与旋转
		///*
		//Qx = [ cos(a/2), (sin(a/2), 0, 0)]
		//Qy = [ cos(b/2), (0, sin(b/2), 0)]
		//Qz = [ cos(c/2), (0, 0, sin(c/2))]
		//And the final quaternion is obtained by Qx * Qy * Qz.
		//*/

		//Quaternion<T> 
		//	Qx(cos(roll/2),sin(roll/2),0,0),
		//	Qy(cos(pitch/2),0,sin(pitch/2),0),
		//	Qz(cos(yaw/2),0,0,sin(yaw/2));

		//*this = Qx * Qy * Qz;

	}

	//this function is not used to get the rotation matrix.
	//just get the matrix for quaternion multiplying.
	matrix<T> getMatrix1() // for q1*q2 = R*q2
	{
		matrix<T> R(4,4);
		R(0,0) =  w;  R(0,1) = -x; R(0,2) = -y; R(0,3) = -z; 
		R(1,0) =  x;  R(1,1) =  w; R(1,2) = -z; R(1,3) =  y; 
		R(2,0) =  y;  R(2,1) =  z; R(2,2) =  w; R(2,3) = -x; 
		R(3,0) =  z;  R(3,1) = -y; R(3,2) =  x; R(3,3) =  w; 

		return R;
	}
	matrix<T> getMatrix2() // for q2*q1 = R*q2
	{
		matrix<T> R(4,4);
		R(0,0) =  w;  R(0,1) = -x; R(0,2) = -y; R(0,3) = -z; 
		R(1,0) =  x;  R(1,1) =  w; R(1,2) =  z; R(1,3) = -y; 
		R(2,0) =  y;  R(2,1) = -z; R(2,2) =  w; R(2,3) =  x; 
		R(3,0) =  z;  R(3,1) =  y; R(3,2) = -x; R(3,3) =  w; 

		return R;
	}



	Quaternion exponential()
	{
		T ww,lenV;
		vector3<T> vv;

		vv = *this;
		lenV = vv.length();

		if (lenV == 0)
		{
			return Quaternion(1,0,0,0);
		}

		ww = cos(lenV);
		vv = vv * sin(lenV)/lenV;

		Quaternion q(ww,vv);

		return q * exp(w);
	}

	Quaternion logarithm()
	{
		T ww,lenQ,lenV;
		vector3<T> vv;

		vv = *this;
		lenV = vv.length();

		if (lenV == 0)
		{
			return Quaternion(1,0,0,0);
		}

		lenQ = this->length();

		Quaternion q(log(lenQ),acos(w/lenQ)*vv/lenV);

		return q ;
	}


};

typedef Quaternion<double> TQuaternionD;
typedef TQuaternionD * PTQuaternionD;



// 型の昇格を行う演算の定義 //Definition of operations to perform the promotion of type

/// @brief クォータニオンの和
DEFINE_PROMOTE_BIND_OPERATOR1( Quaternion, + )

/// @brief クォータニオンと定数の和
DEFINE_PROMOTE_BIND_OPERATOR2( Quaternion, + )

/// @brief 定数とクォータニオンの和
DEFINE_PROMOTE_BIND_OPERATOR3( Quaternion, + )

/// @brief クォータニオンの差
DEFINE_PROMOTE_BIND_OPERATOR1( Quaternion, - )

/// @brief クォータニオンと定数の差
DEFINE_PROMOTE_BIND_OPERATOR2( Quaternion, - )

/// @brief 定数とクォータニオンの差
DEFINE_PROMOTE_BIND_OPERATOR4( Quaternion, - )

/// @brief クォータニオンの積
DEFINE_PROMOTE_BIND_OPERATOR1( Quaternion, * )

/// @brief クォータニオンと定数の積
DEFINE_PROMOTE_BIND_OPERATOR2( Quaternion, * )

/// @brief 定数とクォータニオンの積
DEFINE_PROMOTE_BIND_OPERATOR3( Quaternion, * )

/// @brief クォータニオンの割り算
DEFINE_PROMOTE_BIND_OPERATOR1( Quaternion, / )

/// @brief クォータニオンを定数で割る
DEFINE_PROMOTE_BIND_OPERATOR2( Quaternion, / )






/// @brief 指定されたストリームに，コンテナ内の要素を整形して出力する    //指定されたストリームに，コンテナ内の要素を整形して出力する     指定的流，并输出在所述容器的成形单元
//! 
//! @param[in,out] out … 入力と出力を行うストリーム
//! @param[in]     q   … 3次元ベクトル
//! 
//! @return 入力されたストリーム
//! 
//! @code 出力例
//! ( 1, 2, 3, 4 )
//! @endcode
//! 
template < class T > inline std::ostream &operator <<( std::ostream &out, const Quaternion< T > &q )
{
	out << "( ";
	out << q.w << ", ";
	out << q.x << ", ";
	out << q.y << ", ";
	out << q.z << " )";
	return( out );
}



/// @brief 球面線形補間を行う   //spherical linear interpolation  球面线性插值
//! 
//! @param[in] q1 … 補間もとのクォータニオン1
//! @param[in] q2 … 補間もとのクォータニオン2
//! @param[in] t  … [0,1]の間の数値で，補間点
//! 
//! @return 球面線形補間されたクォータニオン     The spherical linear interpolated quaternion
//! 
template < class T1, class T2 >
const Quaternion< double > interpolate( const Quaternion< T1 > &q1, const Quaternion< T2 > &q2, double t )
{
	typedef Quaternion< double > quaternion_type;

	quaternion_type Q1( q1.unit( ) );
	quaternion_type Q2( q2.unit( ) );

	double dot = Q1.inner( Q2 );

	if( std::abs( dot ) < 1.0e-6 )
	{
		return( Q1 );
	}
	else if( dot < 0.0 )
	{
		double theta = std::acos( dot );

		// 球面線形補間を行う  do spherical linear interpolation
		return( quaternion_type( Q1 * std::sin( theta * ( 1.0 - t ) ) - Q2 * std::sin( theta * t ) ).unit( ) );
	}
	else
	{
		double theta = std::acos( dot );

		// 球面線形補間を行う   do spherical linear interpolation
		return( quaternion_type( Q1 * std::sin( theta * ( 1.0 - t ) ) + Q2 * std::sin( theta * t ) ).unit( ) );
	}
}

template < class T>
const Quaternion< T > interpolateQuaternion( const Quaternion< T > &q1, const Quaternion< T > &q2, double t )
{
	typedef Quaternion< T > quaternion_type;



	quaternion_type Q1( q1.unit( ) );
	quaternion_type Q2( q2.unit( ) );

	quaternion_type q = (Q1.conjugate() * Q2).unit();

	if(q.w < 0) q = -q;

	if( q.w > 0.999999999999 )
	{
		return( Q1 );
	}
	else
	{
		double theta = acos(q.w) * t;
		vector3<T> axis = q;
		axis = axis.unit();

		double c = cos( theta );
		double s = sin( theta );

		T w = static_cast< T >( c );
		T x = static_cast< T >( s * axis.x );
		T y = static_cast< T >( s * axis.y );
		T z = static_cast< T >( s * axis.z );

		q = quaternion_type(w,x,y,z );

		return (Q1 * q).unit();
	}
}

template < class T>
const Quaternion< T > averageQuaternion2(long num, const Quaternion< T > *qList)
{
	typedef Quaternion< T > quaternion_type;

	if ((num <= 0) || (qList == 0)) return quaternion_type(1,0,0,0);

	quaternion_type q(qList[0]);

	for(long i = 1; i< num; i++)
	{
		q = interpolateQuaternion(q,qList[i],1.0f/i);
	}

	return q;
}


template < class T>
const Quaternion< T > averageQuaternion(long num, const Quaternion< T > *qList)
{
	typedef Quaternion< T > quaternion_type;



	if ((num <= 0) || (qList == 0)) return quaternion_type(1,0,0,0);


	if(num == 1) 
		return qList[0];
	else if(num == 2) 
		return interpolateQuaternion(qList[0],qList[1],0.5f);
	else
	{
		quaternion_type *qList1, * qList2;

		long num1 = floor(num/2.0);
		long num2 = ceil(num/2.0);
		qList1 = new quaternion_type[num1];
		qList2 = new quaternion_type[num2];


		for(int i = 0; i < num1; i++)
		{
			qList1[i] = qList[i];
		}
		for(int i = 0; i < num2; i++)
		{
			qList2[i] = qList[num1+i];
		}


		quaternion_type q1 = averageQuaternion(num1, qList1);
		quaternion_type q2 = averageQuaternion(num2, qList2);

		delete [] qList1;
		delete [] qList2;

		return  interpolateQuaternion(q1,q2,num2 * 1.0f/(num1+num2));
	}

}



/// @brief 仮想トラックボールの実装(左手座標系)  //Implementation of a virtual trackball left coordinate system.
//! 
//! @note 以下のソースコードを参考にした
//! 
//! Trackball code:
//! 
//! Implementation of a virtual trackball.
//! Implemented by Gavin Bell, lots of ideas from Thant Tessman and
//!   the August '88 issue of Siggraph's "Computer Graphics," pp. 121-129.
//! 
//! Vector manip code:
//! 
//! Original code from:
//! David M. Ciemiewicz, Mark Grossman, Henry Moreton, and Paul Haeberli
//! 
//! Much mucking with by:
//! Gavin Bell
//! 
//! @param[in] p1             … 回転前の点
//! @param[in] p2             … 回転後の点
//! @param[in] axisX          … トラックボールのX軸
//! @param[in] axisY          … トラックボールのY軸
//! @param[in] axisZ          … トラックボールのZ軸
//! @param[in] trackball_size … トラックボールの半径（デフォルトは0.8）
//! 
//! @return 回転を表すクォータニオン
//! 
template < class T >
const Quaternion< T > track_ball( const vector2< T > &p1, const vector2< T > &p2, const vector3< T > &axisX, const vector3< T > axisY, const vector3< T > axisZ, const typename vector3< T >::value_type &trackball_size )
{
	typedef typename Quaternion< T >::value_type value_type;

	if( p1 == p2 )
	{
		return( Quaternion< T >( 1, 0, 0, 0 ) );
	}

	vector3< T > sp1( p1.x, p1.y, 0 ), sp2( p2.x, p2.y, 0 );
	value_type l, _2 = std::sqrt( value_type( 2.0 ) );

	// 点1の座標を仮想トラックボール上に投影
	l = p1.length( );
	if( l < trackball_size / _2 )
	{
		sp1.z = - std::sqrt( trackball_size * trackball_size - l * l );
	}
	else
	{
		sp1.z = - trackball_size * trackball_size / 2.0 / l;
	}

	// 点2の座標を仮想トラックボール上に投影
	l = p2.length( );
	if( l < trackball_size / _2 )
	{
		sp2.z = - std::sqrt( trackball_size * trackball_size - l * l );
	}
	else
	{
		sp2.z = - trackball_size * trackball_size / 2.0 / l;
	}

	//	sp1 = sp1.unit();
	//	sp2 = sp2.unit();

	// 右手系と左手系でここの外積の向きを反転させる
	//	Vector3<double> axis = (sp2 * sp1).unit();
	vector3< T > axis = ( sp1 * sp2 ).unit( );
	axis = ( axis.x * axisX + axis.y * axisY + axis.z * axisZ ).unit( );

	l = ( sp2 - sp1 ).length( ) / ( 2 * trackball_size );
	//	l = (l < -1.0)? -1.0: l;
	l = l > 1 ? 1: l;

	double phi = std::asin( l );
	//	fprintf(stdout, "axis(%.1f, %.1f, %.1f)   theta = %.1f\n", axis.x, axis.y, axis.z, phi * 180 / PAI);
	//	printf("%.1f\n", phi * 180 / PAI);
	return( Quaternion< T >( std::cos( phi ), std::sin( phi ) * axis ) );
}

/// @brief 仮想トラックボールの実装(左手座標系)
//! 
//! @note 以下のソースコードを参考にした
//! 
//! Trackball code:
//! 
//! Implementation of a virtual trackball.
//! Implemented by Gavin Bell, lots of ideas from Thant Tessman and
//!   the August '88 issue of Siggraph's "Computer Graphics," pp. 121-129.
//! 
//! Vector manip code:
//! 
//! Original code from:
//! David M. Ciemiewicz, Mark Grossman, Henry Moreton, and Paul Haeberli
//! 
//! Much mucking with by:
//! Gavin Bell
//! 
//! @param[in] p1             … 回転前の点
//! @param[in] p2             … 回転後の点
//! @param[in] axisX          … トラックボールのX軸
//! @param[in] axisY          … トラックボールのY軸
//! @param[in] axisZ          … トラックボールのZ軸
//! 
//! @return 回転を表すクォータニオン
//! 
template < class T >
inline const Quaternion< T > track_ball( const vector2< T > &p1, const vector2< T > &p2, const vector3< T > &axisX, const vector3< T > axisY, const vector3< T > axisZ )
{
	return( track_ball( p1, p2, axisX, axisY, axisZ, 0.8 ) );
}


/// @brief 仮想トラックボールの実装(左手座標系)
//! 
//! トラックボールを用いて，任意ベクトルの回転を行う
//! 
//! @param[in] x1             … 回転前のX座標
//! @param[in] y1             … 回転前のY座標
//! @param[in] x2             … 回転後のX座標
//! @param[in] y2             … 回転後のY座標
//! @param[in] axisX          … トラックボールのX軸
//! @param[in] axisY          … トラックボールのY軸
//! @param[in] axisZ          … トラックボールのZ軸
//! @param[in] trackball_size … トラックボールの半径（デフォルトは0.8）
//! 
//! @return 回転を表すクォータニオン
//! 
template < class T >
const Quaternion< T > track_ball( const typename vector3< T >::value_type &x1, const typename vector3< T >::value_type &y1, const typename vector3< T >::value_type &x2, const typename vector3< T >::value_type &y2,
								 const vector3< T > &axisX, const vector3< T > &axisY, const vector3< T > &axisZ, const typename vector3< T >::value_type &trackball_size )
{
	return( track_ball( vector2< T >( x1, y1 ), vector2< T >( x2, y2 ), axisX, axisY, axisZ, trackball_size ) );
}


/// @brief 仮想トラックボールの実装(左手座標系)
//! 
//! トラックボールを用いて，任意ベクトルの回転を行う
//! 
//! @param[in] x1             … 回転前のX座標
//! @param[in] y1             … 回転前のY座標
//! @param[in] x2             … 回転後のX座標
//! @param[in] y2             … 回転後のY座標
//! @param[in] axisX          … トラックボールのX軸
//! @param[in] axisY          … トラックボールのY軸
//! @param[in] axisZ          … トラックボールのZ軸
//! 
//! @return 回転を表すクォータニオン
//! 
template < class T >
const Quaternion< T > track_ball( const typename vector3< T >::value_type &x1, const typename vector3< T >::value_type &y1, const typename vector3< T >::value_type &x2,
								 const typename vector3< T >::value_type &y2, const vector3< T > &axisX, const vector3< T > &axisY, const vector3< T > &axisZ )
{
	return( track_ball( vector2< T >( x1, y1 ), vector2< T >( x2, y2 ), axisX, axisY, axisZ, 0.8 ) );
}





///////////////////////////////////////////////////////////////////////////////////


template < class T >
class DualQuaternion
{
public:
	typedef T value_type;					///< @brief MISTのコンテナ内に格納するデータ型．mist::array< data > の data と同じ
	typedef size_t size_type;				///< @brief 符号なしの整数を表す型．コンテナ内の要素数や，各要素を指定するときなどに利用し，内部的には size_t 型と同じ
	typedef ptrdiff_t difference_type;		///< @brief 符号付きの整数を表す型．コンテナ内の要素数や，各要素を指定するときなどに利用し，内部的には ptrdiff_t 型と同じ
	typedef Quaternion<T> quaternion_type;


public:
	quaternion_type a;
	quaternion_type b;
public:


	//create a dualQuaternion using the two part Quaternion of dualQuaternion.
	DualQuaternion(quaternion_type aa, quaternion_type bb)   //paper "Dual quaternions page 4 left"
	{
		if (aa.inner(bb)> 0.0000000001)
		{
			return;
		}
		a = aa.unit();
		b = bb;
	}

	//create a dualQuaternion using the ratation Quaternion aa, and the translation vector t;
	template<class TT>
	DualQuaternion(quaternion_type aa, vector3<TT> t)
	{
		a=aa.unit();
		b= Quaternion<T>(0,t) * a /2;
	}
	template<class TT>
	//create a dualQuaternion using the rotation angle theta, 
	//and the axis with respect to which is rotated,
	//and the ranslation vector t.
	DualQuaternion(double theta, vector3<TT> axis,vector3<TT> translation)
	{
		a = Quaternion<T>(axis,theta);
		b = Quaternion<T>(0,translattion)*a/2;

	}

	DualQuaternion(DualQuaternion & Q)
	{ 
		a = Q.a;
		b = Q.b;
	}
	DualQuaternion(){};
	
	DualQuaternion operator - ()
	{
		return DualQuaternion(-a,-b);
	}

	template<class TT>
	const DualQuaternion & operator -= (const DualQuaternion<TT> & dualQ)
	{
		a -= dualQ.a;
		b -= dualQ.b;
		return (*this);
	}
	const DualQuaternion & operator += (const DualQuaternion<T> & dualQ)
	{
		a += dualQ.a;
		b += dualQ.b;
		return (*this);
	}
	const DualQuaternion & operator *= (const DualQuaternion<T> & dualQ)
	{
		a *=dualQ.a;
		b = a*dualQ.b + dualQ.a*b;
		return (*this);
	}

	template < class TT >
	const DualQuaternion &operator *=( const TT &aa )
	{

		a *= aa;
		b *= aa;
		return( *this );
	}

	template < class TT >
	const DualQuaternion &operator /=( const TT &aa )
	{

		a /= aa;
		b /= aa;
		return( *this );
	}

	const DualQuaternion<double> inv() const
	{
		double length_=length();
		return (conjugate()/length_* length_);
	}
	double length() const
	{
		return std::sqrt((*this)* this->conjugate());
	}
	const DualQuaternion & operator /= (const DualQuaternion<T> & dualQ)
	{
		return( this->operator *=( dualQ.inv( ) ) );
	}
	const DualQuaternion conjugate( ) const 
	{
		return( DualQuaternion( a.conjugate(),b.conjugate() ) );
	}
	const DualQuaternion< double > unit( ) const
	{
		double length_ = length( );
		return( Quaternion< double >( w / length_, x / length_, y / length_, z / length_ ) );
	}

	template < class TT >
	const vector3< TT > rotate( const vector3< TT > &v ) const
	{

		Quaternion q = a * Quaternion( 0, static_cast< value_type >( v.x ), static_cast< value_type >( v.y ), static_cast< value_type >( v.z ) ) * a.inv( );
		vector v = vector3< TT >( 
			static_cast< TT >( q.x ), 
			static_cast< TT >( q.y ), 
			static_cast< TT >( q.z ) );

		q = 2 * b * a.conjugate();
		vector t = vector3<TT>( static_cast< TT >( q.x ), static_cast< TT >( q.y ), static_cast< TT >( q.z ) ) ;
		return (v + t);
	}



/*================================================================================*\
	Author :JIANG.Zhengang			Date : 2008/07/13			version 1.0
  ================================================================================
	Function :	ConvertFromMatrix

	Description :
		This subroutine is used to convert a transformation matrix to a dualQuaternion.
		where the matrix is described as flowing:
				┌      ┐
				| R  t |
				| 0  1 |
				└      ┘.
		And the rotation information R will be convert to Quaternion of part a,
		and the translation information t will be convet into quternion of part b 
		with using the formula:
		  dualquaternion of transformation = (a , (1+t)*a);

	Return :		void-

	Parameters :

			matrix<T> mat- it must be a 4*4 matrix of transformation.

	Note :

\*================================================================================*/
	
	void ConvertFromMatrix(const matrix<T> & mat)
	{
		if((mat.cols()!=4) || (mat.rows()!=4))
			return;
		Quaternion<T> t(0,mat(0,3),mat(1,3),mat(2,3));
		a.ConvertFromMatrix(mat);
		b = t*a/2;
	}

	vector3<T> getTranslationVector()
	{
		vector3<T> translation;
		Quaternion<T> t= 2*b*a.conjugate();
		translation.x = t.x;
		translation.y = t.y;
		translation.z = t.z;

		return translation;

	}
	Quaternion<T> getRotationQuaternion()
	{
		return a;
	}

	vector3<T> getVectorL()
	{
		vector3<T> l;
		l.x = a.x;		l.y = a.y;		l.z = a.z;		l = l.unit();
		return l;
	}
	vector3<T> getVectorT()
	{
		return getTranslationVector();
	}
	vector3<T> getVectorM()
	{
		vector3<T> m,l,t;
		double theta = acos(a.w);

		l = getVectorL();
		t = getTranslationVector();
		m = (t.outer(l)+l.outer(t.outer(l))*a.w/sin(theta))/2;

		return m;

	}



/*================================================================================*\
	Author :JIANG.Zhengang			Date : 2008/07/14			version 1.0
  ================================================================================
	Function :	ConvertToMatrix

	Description :
		
		This subroutine is used to convert a dualQuaternion to a transformation matrix
		where the matrix is described as flowing:
				┌      ┐
				| R  t |
				| 0  1 |
				└      ┘.

		R,t is given in method ConvertFromMatrix.

	Return :	matrix<T> 

	Parameters :

			

	Note :

\*================================================================================*/
	matrix<T> ConvertToMatrix()
	{
		matrix<T> R = a.ConvertToMatrix() ;

		//b = t*a/2;  =>  t = 2*b*a.inv();
		mist::vector3<T> t = getTranslationVector();

		R(0,3) = t.x;
		R(1,3) = t.y;
		R(2,3) = t.z;
		return R;
	}

	

};

typedef Quaternion<double> QuaternionD;
typedef QuaternionD * PQuaternionD;

typedef DualQuaternion<double> DualQuaternionD;
typedef DualQuaternionD * PDualQuaternionD;

template <class T>
Quaternion<T> getQuaternion( matrix<T> transform)
{
	Quaternion<T> q(0,0,0,0);
	if (transform.size() != 0)
		q.ConvertFromMatrix(transform);
	return q;
}

template <class T>
vector3<T> getTranslation( matrix<T> transform)
{
	vector3<T> v(0,0,0);
	if (transform.size()!=0)
	{
		v.x = transform(0,3);
		v.y = transform(1,3);
		v.z = transform(2,3);
	}
	return v;
}

enum FilterStyle { fsUniform=0,fsGaussian,fsHalfGaussian,fsWeights};


template<class T>
Quaternion<T> GetAverageOfQuaternions(Quaternion<T> * qList, int count)
{
	if(count == 0 || qList == 0)
	{
		return Quaternion<T>(0,0,0,0);
	}
	if( count == 1)
	{
		return qList[0];
	}
	//there are more than two quaternions in the list
	//we will find the average of quaterions by a iterative algrigthm.

	Quaternion<T> q0, qU,logQ0(1,0,0,0), qDash, logQ;
	q0 = qList[0];// the first quaternion will be considered as the initialized average quaternion.
	
	cout << q0.w <<"\t"<< q0.x <<"\t"<< q0.y <<"\t"<< q0.z <<"\n";

	for(;;)
	{
		//transform the others quaternions to the coordinate system of q0;
		//then calulate the sum of them in the tangential plane of point q0.
		qU = Quaternion<T>(0,0,0,0);
		
		for(int i = 0 ; i < count; i++)
		{
			qDash = q0.conjugate()*qList[i];
			if (qDash.w<0) 
			{
				qDash = - qDash;
			}
			qU = qU + qDash.logarithm()-logQ0;
		}

		qU = qU / count;
		if ( qU.length() < 0.00001)
		{
			break;
		}
		else
		{
			qDash = (logQ0 + qU).exponential();
			cout << qDash.w <<"\t"<< qDash.x <<"\t"<< qDash.y <<"\t"<< qDash.z <<"\t"<< qDash.length() <<"\n";
			q0 = q0 * qDash.unit();
			q0 = q0.unit();
			cout << q0.w <<"\t"<< q0.x <<"\t"<< q0.y <<"\t"<< q0.z <<"\n";
		}
	}

	return q0;
}
  
DEFINE_PROMOTE_BIND_OPERATOR1( DualQuaternion, + )

/// @brief クォータニオンと定数の和
DEFINE_PROMOTE_BIND_OPERATOR2( DualQuaternion, + )

/// @brief 定数とクォータニオンの和
DEFINE_PROMOTE_BIND_OPERATOR3( DualQuaternion, + )

/// @brief クォータニオンの差
DEFINE_PROMOTE_BIND_OPERATOR1( DualQuaternion, - )

/// @brief クォータニオンと定数の差
DEFINE_PROMOTE_BIND_OPERATOR2( DualQuaternion, - )

/// @brief 定数とクォータニオンの差
DEFINE_PROMOTE_BIND_OPERATOR4( DualQuaternion, - )

/// @brief クォータニオンの積
DEFINE_PROMOTE_BIND_OPERATOR1( DualQuaternion, * )

/// @brief クォータニオンと定数の積
DEFINE_PROMOTE_BIND_OPERATOR2( DualQuaternion, * )

/// @brief 定数とクォータニオンの積
DEFINE_PROMOTE_BIND_OPERATOR3( DualQuaternion, * )

/// @brief クォータニオンの割り算
DEFINE_PROMOTE_BIND_OPERATOR1( DualQuaternion, / )

/// @brief クォータニオンを定数で割る
DEFINE_PROMOTE_BIND_OPERATOR2( DualQuaternion, / )