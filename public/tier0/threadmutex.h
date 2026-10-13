#ifndef THREADMUTEX_H
#define THREADMUTEX_H

#include "tier0/threadinterlocks.h"
#include "tier0/threadtools.h"

// Detects overlapping use by different threads; it does not wait for an owner.
class CThreadNullVerificationMutex
{
public:
	CThreadNullVerificationMutex() : m_OwnerID( 0 ), m_Depth( 0 ) {}
	CThreadNullVerificationMutex( const CThreadNullVerificationMutex & ) : CThreadNullVerificationMutex() {}

	CThreadNullVerificationMutex &operator=( const CThreadNullVerificationMutex & ) = delete;

	void Lock( const char *pFileName = NULL, int nLine = -1 );
	void Unlock( const char *pFileName = NULL, int nLine = -1 );

private:
	volatile ThreadId_t m_OwnerID;
	int m_Depth;
};

inline void CThreadNullVerificationMutex::Lock( const char *pFileName, int nLine )
{
	ThreadId_t nThreadID = ThreadGetCurrentId();
#ifdef PLATFORM_LINUX
	ThreadId_t nOwnerID = ThreadInterlockedCompareExchange64( (volatile int64 *)&m_OwnerID, nThreadID, 0 );
#else
	ThreadId_t nOwnerID = ThreadInterlockedCompareExchange( &m_OwnerID, nThreadID, 0 );
#endif
	if ( nOwnerID == 0 )
	{
		m_Depth = 1;
	}
	else if ( nOwnerID == nThreadID )
	{
		++m_Depth;
	}
	else
	{
		// Verification must remain enabled in release builds.
		if ( Log_Assert( "CThreadNullVerificationMutex: Thread %llu acquiring null-mutex while thread %llu is still holding it, indicates unexpected thread-safety issue",
			(uint64)nThreadID, (uint64)nOwnerID ) == LR_DEBUGGER )
		{
			DebuggerBreak();
		}
	}
}

inline void CThreadNullVerificationMutex::Unlock( const char *pFileName, int nLine )
{
	if ( m_OwnerID == ThreadGetCurrentId() && --m_Depth == 0 )
	{
#ifdef PLATFORM_LINUX
		ThreadInterlockedExchange64( (volatile int64 *)&m_OwnerID, 0 );
#else
		ThreadInterlockedExchange( &m_OwnerID, 0 );
#endif
	}
}

#endif // THREADMUTEX_H
