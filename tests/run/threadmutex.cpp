// Standalone logic test: replace tier0's assertion sink to observe failures.
#include "tier0/dbg.h"

static LoggingResponse_t TestLogAssert( const char *pFormat, ... );
#undef Log_Assert
#define Log_Assert TestLogAssert
#include "tier0/threadmutex.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <thread>

static int g_nFailures;
static int g_nAssertions;
static uint64 g_nLastThreadID;
static uint64 g_nLastOwnerID;

#define CHECK( expr ) \
	do \
	{ \
		if ( !( expr ) ) \
		{ \
			printf( "%s:%d: CHECK( %s ) failed\n", __FILE__, __LINE__, #expr ); \
			++g_nFailures; \
		} \
	} while ( 0 )

#ifdef PLATFORM_LINUX
PLATFORM_INTERFACE ThreadId_t ThreadGetCurrentId()
{
	return pthread_self();
}
#endif

static LoggingResponse_t TestLogAssert( const char *pFormat, ... )
{
	++g_nAssertions;
	CHECK( strstr( pFormat, "CThreadNullVerificationMutex:" ) != NULL );
	va_list args;
	va_start( args, pFormat );
	g_nLastThreadID = va_arg( args, uint64 );
	g_nLastOwnerID = va_arg( args, uint64 );
	va_end( args );
	return LR_CONTINUE;
}

static void TryFromOtherThread( CThreadNullVerificationMutex &mutex, int nExpectedAssertions )
{
	int nBefore = g_nAssertions;
	std::thread thread( [&mutex]()
	{
		mutex.Lock();
		mutex.Unlock();
	} );
	thread.join();
	CHECK( g_nAssertions == nBefore + nExpectedAssertions );
	if ( nExpectedAssertions )
	{
		CHECK( g_nLastOwnerID == (uint64)ThreadGetCurrentId() );
		CHECK( g_nLastThreadID != g_nLastOwnerID );
		CHECK( g_nLastThreadID != 0 );
	}
}

int main()
{
	CThreadNullVerificationMutex mutex;
	mutex.Unlock();
	TryFromOtherThread( mutex, 0 );

	mutex.Lock( __FILE__, __LINE__ );
	mutex.Lock();
	CHECK( g_nAssertions == 0 );
	TryFromOtherThread( mutex, 1 );

	// A failed acquisition and its matching unlock must leave the owner intact.
	mutex.Unlock();
	TryFromOtherThread( mutex, 1 );
	mutex.Unlock( __FILE__, __LINE__ );
	TryFromOtherThread( mutex, 0 );

	mutex.Lock();
	std::thread nonOwner( [&mutex]() { mutex.Unlock(); } );
	nonOwner.join();
	TryFromOtherThread( mutex, 1 );

	// Copies start unlocked, even if the source is currently held.
	CThreadNullVerificationMutex copy( mutex );
	TryFromOtherThread( copy, 0 );
	TryFromOtherThread( mutex, 1 );
	mutex.Unlock();
	TryFromOtherThread( mutex, 0 );

	{
		CAutoLockT<CThreadNullVerificationMutex> outer( mutex );
		{
			const CThreadNullVerificationMutex &ref = mutex;
			CAutoLockT<CThreadNullVerificationMutex> inner( ref, __FILE__, __LINE__ );
			TryFromOtherThread( mutex, 1 );
		}
		TryFromOtherThread( mutex, 1 );
	}
	TryFromOtherThread( mutex, 0 );

	try
	{
		AUTO_LOCK( mutex );
		throw 1;
	}
	catch ( int )
	{
	}
	TryFromOtherThread( mutex, 0 );
	printf( "threadmutex: %d failures, %d expected assertions\n", g_nFailures, g_nAssertions );
	return g_nFailures != 0;
}
